#include "can_bus/frame.hpp"

namespace can {

namespace {

void push_bits(Bits& out, std::uint32_t value, int count) {
  for (int i = count - 1; i >= 0; --i) out.push_back(static_cast<std::uint8_t>((value >> i) & 1U));
}

// Reads bits while removing stuff bits, exactly mirroring stuff().
class Destuffer {
 public:
  explicit Destuffer(const Bits& wire) : wire_(wire) {}

  // Next real bit. Returns false (with *error set) on truncation or a stuff error.
  bool next(int* bit, DecodeError* error) {
    if (!read_raw(bit, error)) return false;
    if (run_ == 5) {  // five equal bits so far: this one must be a stuff bit
      if (*bit == last_) {
        *error = DecodeError::Stuff;
        return false;
      }
      last_ = *bit;
      run_ = 1;
      if (!read_raw(bit, error)) return false;  // the real bit follows the stuff bit
    }
    if (*bit == last_) {
      ++run_;
    } else {
      last_ = *bit;
      run_ = 1;
    }
    return true;
  }

  // After the last CRC bit a stuff bit may still follow if the CRC ended in five equal bits.
  bool consume_trailing_stuff_bit(DecodeError* error) {
    if (run_ != 5) return true;
    int bit = 0;
    if (!read_raw(&bit, error)) return false;
    if (bit == last_) {
      *error = DecodeError::Stuff;
      return false;
    }
    run_ = 1;
    last_ = bit;
    return true;
  }

  std::size_t position() const { return pos_; }

 private:
  bool read_raw(int* bit, DecodeError* error) {
    if (pos_ >= wire_.size()) {
      *error = DecodeError::Form;  // the frame ended early
      return false;
    }
    *bit = wire_[pos_++];
    return true;
  }

  const Bits& wire_;
  std::size_t pos_{0};
  int last_{-1};
  int run_{0};
};

bool read_field(Destuffer& d, int count, std::uint32_t* value, Bits* region, DecodeError* error) {
  *value = 0;
  for (int i = 0; i < count; ++i) {
    int bit = 0;
    if (!d.next(&bit, error)) return false;
    *value = (*value << 1) | static_cast<std::uint32_t>(bit);
    if (region) region->push_back(static_cast<std::uint8_t>(bit));
  }
  return true;
}

}  // namespace

bool Frame::operator==(const Frame& o) const {
  if (id != o.id || dlc != o.dlc) return false;
  for (int i = 0; i < dlc; ++i) {
    if (data[i] != o.data[i]) return false;
  }
  return true;
}

std::uint16_t crc15(const Bits& bits) {
  std::uint16_t reg = 0;
  for (std::uint8_t b : bits) {
    const bool next = (b != 0) != (((reg >> 14) & 1U) != 0);
    reg = static_cast<std::uint16_t>((reg << 1) & 0x7FFF);
    if (next) reg ^= 0x4599;
  }
  return reg;
}

Bits crc_region(const Frame& f) {
  Bits bits;
  bits.push_back(0);          // SOF (dominant)
  push_bits(bits, f.id, 11);  // identifier
  bits.push_back(0);          // RTR: data frame
  bits.push_back(0);          // IDE: standard identifier
  bits.push_back(0);          // r0 (reserved)
  push_bits(bits, f.dlc, 4);  // data length code
  for (int i = 0; i < f.dlc; ++i) push_bits(bits, f.data[i], 8);
  return bits;
}

Bits stuff(const Bits& in) {
  Bits out;
  out.reserve(in.size() + in.size() / 4);
  int last = -1;
  int run = 0;
  for (std::uint8_t b : in) {
    out.push_back(b);
    if (b == last) {
      ++run;
    } else {
      last = b;
      run = 1;
    }
    if (run == 5) {  // five equal bits: insert the complement, which starts a new run
      const std::uint8_t complement = b ? 0 : 1;
      out.push_back(complement);
      last = complement;
      run = 1;
    }
  }
  return out;
}

bool destuff(const Bits& in, Bits& out) {
  out.clear();
  int last = -1;
  int run = 0;
  bool expect_stuff = false;
  for (std::uint8_t b : in) {
    if (expect_stuff) {
      if (b == last) return false;  // six equal bits
      expect_stuff = false;
      last = b;
      run = 1;
      continue;
    }
    out.push_back(b);
    if (b == last) {
      ++run;
    } else {
      last = b;
      run = 1;
    }
    if (run == 5) expect_stuff = true;
  }
  return true;
}

Bits encode(const Frame& frame) {
  Bits region = crc_region(frame);
  push_bits(region, crc15(region), 15);
  Bits wire = stuff(region);
  for (int i = 0; i < kTrailerBits; ++i) wire.push_back(1);  // delimiters, ACK slot, EOF, IFS
  return wire;
}

const char* decode_error_name(DecodeError e) {
  switch (e) {
    case DecodeError::None:
      return "none";
    case DecodeError::Stuff:
      return "stuff";
    case DecodeError::Crc:
      return "crc";
    case DecodeError::Form:
      return "form";
    case DecodeError::Dlc:
      return "dlc";
    case DecodeError::Unsupported:
      return "unsupported";
  }
  return "?";
}

DecodeResult decode(const Bits& wire) {
  DecodeResult result;
  Destuffer d(wire);
  Bits region;
  DecodeError err = DecodeError::None;
  std::uint32_t v = 0;

  auto fail = [&](DecodeError e) {
    result.error = e;
    return result;
  };

  if (!read_field(d, 1, &v, &region, &err)) return fail(err);
  if (v != 0) return fail(DecodeError::Form);  // SOF must be dominant

  if (!read_field(d, 11, &v, &region, &err)) return fail(err);
  result.frame.id = static_cast<std::uint16_t>(v);

  if (!read_field(d, 1, &v, &region, &err)) return fail(err);  // RTR
  if (v != 0) return fail(DecodeError::Unsupported);
  if (!read_field(d, 1, &v, &region, &err)) return fail(err);  // IDE
  if (v != 0) return fail(DecodeError::Unsupported);
  if (!read_field(d, 1, &v, &region, &err)) return fail(err);  // r0

  if (!read_field(d, 4, &v, &region, &err)) return fail(err);
  if (v > static_cast<std::uint32_t>(kMaxDlc)) return fail(DecodeError::Dlc);
  result.frame.dlc = static_cast<std::uint8_t>(v);

  for (int i = 0; i < result.frame.dlc; ++i) {
    if (!read_field(d, 8, &v, &region, &err)) return fail(err);
    result.frame.data[i] = static_cast<std::uint8_t>(v);
  }

  std::uint32_t received_crc = 0;
  if (!read_field(d, 15, &received_crc, nullptr, &err)) return fail(err);
  if (received_crc != crc15(region)) return fail(DecodeError::Crc);

  if (!d.consume_trailing_stuff_bit(&err)) return fail(err);
  // CRC delimiter: a plain recessive bit, not stuffed.
  const std::size_t pos = d.position();
  if (pos >= wire.size() || wire[pos] != 1) return fail(DecodeError::Form);
  return result;
}

}  // namespace can
