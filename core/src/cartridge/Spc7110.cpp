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

#include "snes/core/Spc7110.hpp"

namespace snes::core {
void Spc7110::power() {
  reset();
}

void Spc7110::reset() {
  r4801 = 0x00;
  r4802 = 0x00;
  r4803 = 0x00;
  r4804 = 0x00;
  r4805 = 0x00;
  r4806 = 0x00;
  r4807 = 0x00;
  r4808 = 0x00;
  r4809 = 0x00;
  r480a = 0x00;
  r480b = 0x00;
  r480c = 0x00;

  decomp.reset();

  r4811 = 0x00;
  r4812 = 0x00;
  r4813 = 0x00;
  r4814 = 0x00;
  r4815 = 0x00;
  r4816 = 0x00;
  r4817 = 0x00;
  r4818 = 0x00;

  r481x = 0x00;
  r4814_latch = false;
  r4815_latch = false;

  r4820 = 0x00;
  r4821 = 0x00;
  r4822 = 0x00;
  r4823 = 0x00;
  r4824 = 0x00;
  r4825 = 0x00;
  r4826 = 0x00;
  r4827 = 0x00;
  r4828 = 0x00;
  r4829 = 0x00;
  r482a = 0x00;
  r482b = 0x00;
  r482c = 0x00;
  r482d = 0x00;
  r482e = 0x00;
  r482f = 0x00;

  r4830 = 0x00;
  mmio_write(0x4831, 0);
  mmio_write(0x4832, 1);
  mmio_write(0x4833, 2);
  r4834 = 0x00;

  r4840 = 0x00;
  r4841 = 0x00;
  r4842 = 0x00;

  rtc_state = RTCS_Inactive;
  rtc_mode  = RTCM_Linear;
  rtc_index = 0;
}

unsigned Spc7110::datarom_addr(unsigned addr) {
  const size_t size = rom_.size() > 0x500000 ? rom_.size() - 0x200000 : rom_.size() - 0x100000;
  return 0x100000 + addr % size;
}

unsigned Spc7110::data_pointer()   { return r4811 + (r4812 << 8) + (r4813 << 16); }
unsigned Spc7110::data_adjust()    { return r4814 + (r4815 << 8); }
unsigned Spc7110::data_increment() { return r4816 + (r4817 << 8); }
void Spc7110::set_data_pointer(unsigned addr) { r4811 = addr; r4812 = addr >> 8; r4813 = addr >> 16; }
void Spc7110::set_data_adjust(unsigned addr)  { r4814 = addr; r4815 = addr >> 8; }



uint8_t Spc7110::mmio_read(unsigned addr, uint8_t openBus) {
  addr &= 0xffff;
  if (addr >= 0x4840 && !hasRtc_) return openBus;

  switch(addr) {
    //==================
    //decompression unit
    //==================

    case 0x4800: {
      uint16_t counter = (r4809 + (r480a << 8));
      counter--;
      r4809 = counter;
      r480a = counter >> 8;
      return decomp.read();
    }
    case 0x4801: return r4801;
    case 0x4802: return r4802;
    case 0x4803: return r4803;
    case 0x4804: return r4804;
    case 0x4805: return r4805;
    case 0x4806: return r4806;
    case 0x4807: return r4807;
    case 0x4808: return r4808;
    case 0x4809: return r4809;
    case 0x480a: return r480a;
    case 0x480b: return r480b;
    case 0x480c: {
      uint8_t status = r480c;
      r480c &= 0x7f;
      return status;
    }

    //==============
    //data port unit
    //==============

    case 0x4810: {
      if(r481x != 0x07) return 0x00;

      unsigned addr = data_pointer();
      unsigned adjust = data_adjust();
      if(r4818 & 8) adjust = (int16_t)adjust;  //16-bit sign extend

      unsigned adjustaddr = addr;
      if(r4818 & 2) {
        adjustaddr += adjust;
        set_data_adjust(adjust + 1);
      }

      uint8_t data = RomRead(datarom_addr(adjustaddr));
      if(!(r4818 & 2)) {
        unsigned increment = (r4818 & 1) ? data_increment() : 1;
        if(r4818 & 4) increment = (int16_t)increment;  //16-bit sign extend

        if((r4818 & 16) == 0) {
          set_data_pointer(addr + increment);
        } else {
          set_data_adjust(adjust + increment);
        }
      }

      return data;
    }
    case 0x4811: return r4811;
    case 0x4812: return r4812;
    case 0x4813: return r4813;
    case 0x4814: return r4814;
    case 0x4815: return r4815;
    case 0x4816: return r4816;
    case 0x4817: return r4817;
    case 0x4818: return r4818;
    case 0x481a: {
      if(r481x != 0x07) return 0x00;

      unsigned addr = data_pointer();
      unsigned adjust = data_adjust();
      if(r4818 & 8) adjust = (int16_t)adjust;  //16-bit sign extend

      uint8_t data = RomRead(datarom_addr(addr + adjust));
      if((r4818 & 0x60) == 0x60) {
        if((r4818 & 16) == 0) {
          set_data_pointer(addr + adjust);
        } else {
          set_data_adjust(adjust + adjust);
        }
      }

      return data;
    }

    //=========
    //math unit
    //=========

    case 0x4820: return r4820;
    case 0x4821: return r4821;
    case 0x4822: return r4822;
    case 0x4823: return r4823;
    case 0x4824: return r4824;
    case 0x4825: return r4825;
    case 0x4826: return r4826;
    case 0x4827: return r4827;
    case 0x4828: return r4828;
    case 0x4829: return r4829;
    case 0x482a: return r482a;
    case 0x482b: return r482b;
    case 0x482c: return r482c;
    case 0x482d: return r482d;
    case 0x482e: return r482e;
    case 0x482f: {
      uint8_t status = r482f;
      r482f &= 0x7f;
      return status;
    }

    //===================
    //memory mapping unit
    //===================

    case 0x4830: return r4830;
    case 0x4831: return r4831;
    case 0x4832: return r4832;
    case 0x4833: return r4833;
    case 0x4834: return r4834;

    //====================
    //real-time clock unit
    //====================

    case 0x4840: return r4840;
    case 0x4841: {
      if(rtc_state == RTCS_Inactive || rtc_state == RTCS_ModeSelect) return 0x00;

      r4842 = 0x80;
      uint8_t data = rtc_[rtc_index];
      rtc_index = (rtc_index + 1) & 15;
      return data;
    }
    case 0x4842: {
      uint8_t status = r4842;
      r4842 &= 0x7f;
      return status;
    }
  }

  return openBus;
}

void Spc7110::mmio_write(unsigned addr, uint8_t data) {
  addr &= 0xffff;
  if (addr >= 0x4840 && !hasRtc_) return;

  switch(addr) {
    //==================
    //decompression unit
    //==================

    case 0x4801: r4801 = data; break;
    case 0x4802: r4802 = data; break;
    case 0x4803: r4803 = data; break;
    case 0x4804: r4804 = data; break;
    case 0x4805: r4805 = data; break;
    case 0x4806: {
      r4806 = data;

      unsigned table   = (r4801 + (r4802 << 8) + (r4803 << 16));
      unsigned index   = (r4804 << 2);
      //unsigned length  = (r4809 + (r480a << 8));
      unsigned mode    = RomRead(datarom_addr(table + index));
      unsigned offset  = (RomRead(datarom_addr(table + index + 1)) << 16)
                       + (RomRead(datarom_addr(table + index + 2)) <<  8)
                       + (RomRead(datarom_addr(table + index + 3)) <<  0);

      decomp.init(mode, offset, mode < 3 ? (r4805 + (r4806 << 8)) << mode : 0);
      r480c = 0x80;
    } break;

    case 0x4807: r4807 = data; break;
    case 0x4808: r4808 = data; break;
    case 0x4809: r4809 = data; break;
    case 0x480a: r480a = data; break;
    case 0x480b: r480b = data; break;

    //==============
    //data port unit
    //==============

    case 0x4811: r4811 = data; r481x |= 0x01; break;
    case 0x4812: r4812 = data; r481x |= 0x02; break;
    case 0x4813: r4813 = data; r481x |= 0x04; break;
    case 0x4814: {
      r4814 = data;
      r4814_latch = true;
      if(!r4815_latch) break;
      if(!(r4818 & 2)) break;
      if(r4818 & 0x10) break;

      if((r4818 & 0x60) == 0x20) {
        unsigned increment = data_adjust() & 0xff;
        if(r4818 & 8) increment = (int8_t)increment;  //8-bit sign extend
        set_data_pointer(data_pointer() + increment);
      } else if((r4818 & 0x60) == 0x40) {
        unsigned increment = data_adjust();
        if(r4818 & 8) increment = (int16_t)increment;  //16-bit sign extend
        set_data_pointer(data_pointer() + increment);
      }
    } break;
    case 0x4815: {
      r4815 = data;
      r4815_latch = true;
      if(!r4814_latch) break;
      if(!(r4818 & 2)) break;
      if(r4818 & 0x10) break;

      if((r4818 & 0x60) == 0x20) {
        unsigned increment = data_adjust() & 0xff;
        if(r4818 & 8) increment = (int8_t)increment;  //8-bit sign extend
        set_data_pointer(data_pointer() + increment);
      } else if((r4818 & 0x60) == 0x40) {
        unsigned increment = data_adjust();
        if(r4818 & 8) increment = (int16_t)increment;  //16-bit sign extend
        set_data_pointer(data_pointer() + increment);
      }
    } break;
    case 0x4816: r4816 = data; break;
    case 0x4817: r4817 = data; break;
    case 0x4818: {
      if(r481x != 0x07) break;

      r4818 = data;
      r4814_latch = r4815_latch = false;
    } break;

    //=========
    //math unit
    //=========

    case 0x4820: r4820 = data; break;
    case 0x4821: r4821 = data; break;
    case 0x4822: r4822 = data; break;
    case 0x4823: r4823 = data; break;
    case 0x4824: r4824 = data; break;
    case 0x4825: {
      r4825 = data;

      if(r482e & 1) {
        //signed 16-bit x 16-bit multiplication
        int16_t r0 = (int16_t)(r4824 + (r4825 << 8));
        int16_t r1 = (int16_t)(r4820 + (r4821 << 8));

        signed result = r0 * r1;
        r4828 = result;
        r4829 = result >> 8;
        r482a = result >> 16;
        r482b = result >> 24;
      } else {
        //unsigned 16-bit x 16-bit multiplication
        uint16_t r0 = (uint16_t)(r4824 + (r4825 << 8));
        uint16_t r1 = (uint16_t)(r4820 + (r4821 << 8));

        uint32_t result = uint32_t(r0) * r1;
        r4828 = result;
        r4829 = result >> 8;
        r482a = result >> 16;
        r482b = result >> 24;
      }

      r482f = 0x80;
    } break;
    case 0x4826: r4826 = data; break;
    case 0x4827: {
      r4827 = data;

      if(r482e & 1) {
        //signed 32-bit x 16-bit division
        int32_t dividend = (int32_t)(r4820 + (r4821 << 8) + (r4822 << 16) + (uint32_t(r4823) << 24));
        int16_t divisor  = (int16_t)(r4826 + (r4827 << 8));

        int32_t quotient;
        int16_t remainder;

        if(divisor) {
          quotient  = (int32_t)(int64_t(dividend) / divisor);
          remainder = (int32_t)(int64_t(dividend) % divisor);
        } else {
          //illegal division by zero
          quotient  = 0;
          remainder = dividend & 0xffff;
        }

        r4828 = quotient;
        r4829 = quotient >> 8;
        r482a = quotient >> 16;
        r482b = quotient >> 24;

        r482c = remainder;
        r482d = remainder >> 8;
      } else {
        //unsigned 32-bit x 16-bit division
        uint32_t dividend = (uint32_t)(r4820 + (r4821 << 8) + (r4822 << 16) + (uint32_t(r4823) << 24));
        uint16_t divisor  = (uint16_t)(r4826 + (r4827 << 8));

        uint32_t quotient;
        uint16_t remainder;

        if(divisor) {
          quotient  = (uint32_t)(dividend / divisor);
          remainder = (uint16_t)(dividend % divisor);
        } else {
          //illegal division by zero
          quotient  = 0;
          remainder = dividend & 0xffff;
        }

        r4828 = quotient;
        r4829 = quotient >> 8;
        r482a = quotient >> 16;
        r482b = quotient >> 24;

        r482c = remainder;
        r482d = remainder >> 8;
      }

      r482f = 0x80;
    } break;

    case 0x482e: {
      //reset math unit
      r4820 = r4821 = r4822 = r4823 = 0;
      r4824 = r4825 = r4826 = r4827 = 0;
      r4828 = r4829 = r482a = r482b = 0;
      r482c = r482d = 0;

      r482e = data;
    } break;

    //===================
    //memory mapping unit
    //===================

    case 0x4830: r4830 = data; break;

    case 0x4831: {
      r4831 = data;
      dx_offset = datarom_addr((data & 7) * 0x100000);
    } break;

    case 0x4832: {
      r4832 = data;
      ex_offset = datarom_addr((data & 7) * 0x100000);
    } break;

    case 0x4833: {
      r4833 = data;
      fx_offset = datarom_addr((data & 7) * 0x100000);
    } break;

    case 0x4834: r4834 = data; break;

    //====================
    //real-time clock unit
    //====================

    case 0x4840: {
      r4840 = data;
      if(!(r4840 & 1)) {
        //disable RTC
        rtc_state = RTCS_Inactive;
        update_time();
      } else {
        //enable RTC
        r4842 = 0x80;
        rtc_state = RTCS_ModeSelect;
      }
    } break;

    case 0x4841: {
      r4841 = data;

      switch(rtc_state) {
        case RTCS_ModeSelect: {
          if(data == RTCM_Linear || data == RTCM_Indexed) {
            r4842 = 0x80;
            rtc_state = RTCS_IndexSelect;
            rtc_mode  = (RTC_Mode)data;
            rtc_index = 0;
          }
        } break;

        case RTCS_IndexSelect: {
          r4842 = 0x80;
          rtc_index = data & 15;
          if(rtc_mode == RTCM_Linear) rtc_state = RTCS_Write;
        } break;

        case RTCS_Write: {
          r4842 = 0x80;

          //control register 0
          if(rtc_index == 13) {
            update_time();
            //increment second counter
            if(data & 2) update_time(+1);

            //round minute counter
            if(data & 8) {
              update_time();

              unsigned second = rtc_[ 0] + rtc_[ 1] * 10;
              //clear seconds
              rtc_[0] = 0;
              rtc_[1] = 0;

              if(second >= 30) update_time(+60);
            }
          }

          //control register 2
          if(rtc_index == 15) {
            update_time();
            //disable timer and clear second counter
            if((data & 1) && !(rtc_[15] & 1)) {
              update_time();

              //clear seconds
              rtc_[0] = 0;
              rtc_[1] = 0;
            }

            //disable timer
            if((data & 2) && !(rtc_[15] & 2)) {
              update_time();
            }
          }

          rtc_[rtc_index] = data & 15;
          rtc_index = (rtc_index + 1) & 15;
        } break;

		case RTCS_Inactive: {
		} break;
      } //switch(rtc_state)
    } break;
  }
}



}
