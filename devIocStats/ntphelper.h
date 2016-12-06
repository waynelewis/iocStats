#ifndef NTPHELPER_H
#define NTPHELPER_H

#include <vector>
#include <string>
#include <map>
#include <sstream>

typedef std::map<std::string, std::string> ntp_peer_data_t;

template<typename V>
V ntp_peer_as(const ntp_peer_data_t& map,
              const std::string& key,
              const V& def)
{
    ntp_peer_data_t::const_iterator it = map.find(key);
    if(it==map.end())
        return def;
    std::istringstream strm(it->second);
    V ret;
    strm >> ret;
    if(strm.fail() || !strm.eof()) // on error or not all consumed
        return def; // return default on parse error (TODO: make some noise?)
    return ret;
}

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
