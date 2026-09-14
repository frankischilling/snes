// snes emulator
// core/src/audio/Srtc.cpp
// Cartridge real-time clock commands and persistent calendar state.

#include "snes/core/Srtc.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <utility>

namespace snes::core {
namespace {
using namespace std::chrono;

year_month_day Date(const std::array<uint8_t, 13>& digits) {
    return year(1000 + digits[9] + digits[10] * 10 + digits[11] * 100) /
        month(digits[8]) / day(digits[6] + digits[7] * 10);
}

void Encode(std::array<uint8_t, 13>& digits, sys_seconds time) {
    const auto days = floor<std::chrono::days>(time);
    const year_month_day date(days);
    const hh_mm_ss tod(time - days);
    const auto number = [&](unsigned offset, unsigned value) {
        digits[offset] = static_cast<uint8_t>(value % 10);
        digits[offset + 1] = static_cast<uint8_t>(value / 10);
    };
    number(0, static_cast<unsigned>(tod.seconds().count()));
    number(2, static_cast<unsigned>(tod.minutes().count()));
    number(4, static_cast<unsigned>(tod.hours().count()));
    number(6, unsigned(date.day()));
    digits[8] = static_cast<uint8_t>(unsigned(date.month()));
    const unsigned yearValue = static_cast<unsigned>(int(date.year()) - 1000) % 1600;
    number(9, yearValue % 100);
    digits[11] = static_cast<uint8_t>(yearValue / 100);
    digits[12] = static_cast<uint8_t>(weekday(days).c_encoding());
}
}

Srtc::Srtc(Clock clock) : clock_(std::move(clock)) {
    if (!clock_) clock_ = [] {
        return std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    };
    timestamp_ = clock_();
    // Start from the host's local calendar; subsequent updates use elapsed seconds.
    const std::time_t hostTime = static_cast<std::time_t>(timestamp_);
    std::tm local{};
#ifdef _WIN32
    const bool valid = localtime_s(&local, &hostTime) == 0;
#else
    const auto* converted = std::localtime(&hostTime);
    const bool valid = converted != nullptr;
    if (valid) local = *converted;
#endif
    if (valid) {
        using namespace std::chrono;
        Encode(digits_, sys_days(year(local.tm_year + 1900) / month(local.tm_mon + 1) / day(local.tm_mday)) +
            hours(local.tm_hour) + minutes(local.tm_min) + seconds(local.tm_sec));
    }
}

void Srtc::SetWeekday() {
    const auto date = Date(digits_);
    if (date.ok()) digits_[12] = static_cast<uint8_t>(std::chrono::weekday(std::chrono::sys_days(date)).c_encoding());
}

void Srtc::Update() {
    using namespace std::chrono;
    const int64_t now = clock_();
    const auto date = Date(digits_);
    const unsigned second = digits_[0] + digits_[1] * 10;
    const unsigned minute = digits_[2] + digits_[3] * 10;
    const unsigned hour = digits_[4] + digits_[5] * 10;
    // Ignore clock rollback and implausible timestamps from damaged save files.
    if (now >= timestamp_ && timestamp_ >= 0 && now - timestamp_ <= 315576000000LL &&
        date.ok() && second < 60 && minute < 60 && hour < 24) {
        Encode(digits_, sys_days(date) + hours(hour) + minutes(minute) + seconds(second) + seconds(now - timestamp_));
    }
    timestamp_ = now;
}

uint8_t Srtc::Read() {
    if (mode_ != Mode::Read) return 0;
    if (index_ == -1) {
        Update();
        latched_ = digits_;
        index_ = 0;
        return 15;
    }
    if (index_ == 13) {
        index_ = -1;
        return 15;
    }
    return latched_[index_++];
}

void Srtc::Write(uint8_t value) {
    value &= 15;
    if (value == 13) { mode_ = Mode::Read; index_ = -1; return; }
    if (value == 14) { mode_ = Mode::Command; return; }
    if (value == 15) return;
    if (mode_ == Mode::Command) {
        mode_ = value == 0 ? Mode::Write : Mode::Idle;
        index_ = value == 0 ? 0 : -1;
        if (value == 4) digits_.fill(0);
        timestamp_ = clock_();
    } else if (mode_ == Mode::Write && index_ >= 0 && index_ < 12) {
        digits_[index_++] = value;
        if (index_ == 12) { SetWeekday(); timestamp_ = clock_(); }
    }
}

std::array<uint8_t, Srtc::SaveSize> Srtc::Save() {
    if (mode_ != Mode::Write || index_ >= 12) Update();
    std::array<uint8_t, SaveSize> data{};
    std::copy(digits_.begin(), digits_.end(), data.begin());
    data[13] = 'R'; data[14] = 'T'; data[15] = 'C';
    for (unsigned i = 0; i < 8; ++i) data[16 + i] = static_cast<uint8_t>(uint64_t(timestamp_) >> (8 * i));
    return data;
}

bool Srtc::Load(std::span<const uint8_t> data) {
    if (data.size() != SaveSize || data[13] != 'R' || data[14] != 'T' || data[15] != 'C' ||
        std::any_of(data.begin(), data.begin() + 13, [](uint8_t digit) { return digit > 15; }) ||
        (data[23] & 0x80)) return false;
    uint64_t timestamp = 0;
    for (unsigned i = 0; i < 8; ++i) timestamp |= uint64_t(data[16 + i]) << (8 * i);
    timestamp_ = static_cast<int64_t>(timestamp);
    std::copy_n(data.begin(), digits_.size(), digits_.begin());
    Update();
    index_ = -1;
    mode_ = Mode::Read;
    return true;
}

}
