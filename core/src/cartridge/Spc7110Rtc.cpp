#include "snes/core/Spc7110.hpp"

#include <algorithm>
#include <bit>
#include <ctime>
#include <stdexcept>

namespace snes::core {
using namespace std::chrono;

Spc7110::Spc7110(std::span<const uint8_t> rom, bool rtc, Clock clock)
    : rom_(rom), hasRtc_(rtc), clock_(std::move(clock)) {
    if (rom.size() <= 0x100000) throw std::invalid_argument("SPC7110 requires a data ROM after the first MiB");
    if (!clock_) clock_ = [] { return duration_cast<seconds>(system_clock::now().time_since_epoch()).count(); };
    timestamp_ = clock_();
    decomp.SetRom(rom);
    reset();
    const std::time_t hostTime = static_cast<std::time_t>(timestamp_);
    std::tm local{};
#ifdef _WIN32
    const bool valid = localtime_s(&local, &hostTime) == 0;
#else
    const auto* time = std::localtime(&hostTime);
    const bool valid = time != nullptr;
    if (valid) local = *time;
#endif
    if (valid) EncodeTime(sys_days(year(local.tm_year + 1900) / month(local.tm_mon + 1) / day(local.tm_mday)) +
                          hours(local.tm_hour) + minutes(local.tm_min) + seconds(local.tm_sec), local.tm_wday);
}

void Spc7110::EncodeTime(sys_seconds time, unsigned weekday) {
    const auto days = floor<std::chrono::days>(time);
    const year_month_day date(days);
    const hh_mm_ss tod(time - days);
    const auto pair = [&](unsigned offset, unsigned value) {
        rtc_[offset] = uint8_t(value % 10); rtc_[offset + 1] = uint8_t(value / 10);
    };
    pair(0, unsigned(tod.seconds().count())); pair(2, unsigned(tod.minutes().count()));
    pair(4, unsigned(tod.hours().count())); pair(6, unsigned(date.day()));
    pair(8, unsigned(date.month())); pair(10, unsigned(int(date.year()) % 100));
    rtc_[12] = uint8_t(weekday % 7);
}

void Spc7110::update_time(int offset) {
    const int64_t now = clock_();
    const auto pair = [&](unsigned i) { return unsigned(rtc_[i] + 10 * rtc_[i + 1]); };
    const unsigned y = pair(10);
    const year_month_day date(year(int(y + (y >= 90 ? 1900 : 2000))), month(pair(8)), day(pair(6)));
    const int64_t elapsed = now >= timestamp_ && timestamp_ >= 0 && now - timestamp_ <= 315576000000LL ? now - timestamp_ : 0;
    if (!(rtc_[13] & 1) && !(rtc_[15] & 3) && date.ok() && pair(0) < 60 && pair(2) < 60 && pair(4) < 24) {
        const auto start = sys_days(date) + hours(pair(4)) + minutes(pair(2)) + seconds(pair(0));
        const auto end = start + seconds(elapsed + offset);
        const auto days = (floor<std::chrono::days>(end) - sys_days(date)).count();
        EncodeTime(end, unsigned((rtc_[12] + days) % 7));
    }
    timestamp_ = now;
}

std::vector<uint8_t> Spc7110::SaveRtc() {
    if (!hasRtc_) return {};
    update_time();
    std::vector<uint8_t> data(rtc_.begin(), rtc_.end());
    const auto stamp = uint64_t(timestamp_);
    for (unsigned i = 0; i < 8; ++i) data.push_back(uint8_t(stamp >> (8 * i)));
    return data;
}

bool Spc7110::LoadRtc(std::span<const uint8_t> data) {
    if (!hasRtc_ || data.size() != 24 ||
        std::any_of(data.begin(), data.begin() + 16, [](uint8_t byte) { return byte > 15; })) return false;
    uint64_t stamp = 0;
    for (unsigned i = 0; i < 8; ++i) stamp |= uint64_t(data[16 + i]) << (8 * i);
    if (stamp > uint64_t(INT64_MAX)) return false;
    std::copy_n(data.begin(), 16, rtc_.begin());
    timestamp_ = int64_t(stamp);
    update_time();
    return true;
}
} // namespace snes::core
