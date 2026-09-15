/*****
 * SPC7110 emulator - version 0.03 (2008-08-10)
 * Copyright (c) 2008, byuu and neviksti
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * The software is provided "as is" and the author disclaims all warranties
 * with regard to this software including all implied warranties of
 * merchantibility and fitness, in no event shall the author be liable for
 * any special, direct, indirect, or consequential damages or any damages
 * whatsoever resulting from loss of use, data or profits, whether in an
 * action of contract, negligence or other tortious action, arising out of
 * or in connection with the use or performance of this software.
 *****/

#pragma once
#include <array>
#include <cstdint>
#include <span>

namespace snes::core {
class Spc7110Decoder {
public:
  uint8_t read();
  void init(unsigned mode, unsigned offset, unsigned index);
  void reset();

  Spc7110Decoder();
  void SetRom(std::span<const uint8_t> rom) { rom_ = rom; }

  std::span<const uint8_t> rom_;
  uint8_t val=0, in=0, span=0, buffer_index=0;
  uint32_t out=0, out0=0, out1=0, inverts=0, lps=0;
  int in_count=0;
  unsigned pixelorder[16]{}, realorder[16]{};
  uint8_t bitplanebuffer[16]{};
  unsigned decomp_mode;
  unsigned decomp_offset;

  //read() will spool chunks half the size of decomp_buffer_size
  enum { decomp_buffer_size = 64 }; //must be >= 64, and must be a power of two
  std::array<uint8_t, decomp_buffer_size> decomp_buffer{};
  unsigned decomp_buffer_rdoffset;
  unsigned decomp_buffer_wroffset;
  unsigned decomp_buffer_length;

  void write(uint8_t data);
  uint8_t dataread();

  void mode0(bool init);
  void mode1(bool init);
  void mode2(bool init);

  static const uint8_t evolution_table[53][4];
  static const uint8_t mode2_context_table[32][2];

  struct ContextState {
    uint8_t index;
    uint8_t invert;
  } context[32];

  uint8_t probability(unsigned n);
  uint8_t next_lps(unsigned n);
  uint8_t next_mps(unsigned n);
  uint8_t toggle_invert(unsigned n);

  unsigned morton16[2][256];
  unsigned morton32[4][256];
  unsigned morton_2x8(unsigned data);
  unsigned morton_4x8(unsigned data);
};


}
