#ifndef NTPHELPER_H
#define NTPHELPER_H

#include <vector>
#include <string>
#include <map>

typedef std::map<std::string, std::string> ntp_peer_data_t;

ntp_peer_data_t ntp_parse_peer_data(const std::string& e);

struct NTPAssembler {
    typedef std::vector<unsigned char> buffer_t;
    buffer_t buffer;
    typedef std::vector<bool> mask_t;
    mask_t buffer_mask;
    bool seen_last;

    NTPAssembler() : buffer(), seen_last(false) {}
    void add(size_t offset, size_t len, bool more, const void *b);
    bool done() const;

    std::string tostring() const { return std::string(buffer.begin(), buffer.end()); }
};

#endif // NTPHELPER_H
