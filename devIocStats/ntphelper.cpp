
#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <vector>

#include <errlog.h>

#include <ntphelper.h>

extern "C" int devntpParseErrors;
int devntpParseErrors = 0;

// anonymous namespace is C++ equivalent to static global in C
namespace {

typedef std::vector<std::string> lines_t;

// split long string at comma ','
void split(const std::string& inp, lines_t& out)
{
    size_t pos = 0, end;

    while(true) {
        end = inp.find_first_of(',', pos);

        out.push_back(inp.substr(pos, end!=inp.npos ? end-pos : inp.npos));
        if(end==inp.npos)
            break;
        else
            pos = end+1;
    }
}

// remove leading and trailing whitespace
// throw runtime_error if the result is
std::string strip(const std::string& s)
{
    size_t start = s.find_first_not_of(" \t\n\r"),
           end   = s.find_last_not_of(" \t\n\r");

    if(start==s.npos || start>=end)
        throw std::runtime_error("stripped string to zero length");
    return s.substr(start, end-start+1);
}

} // namespace

ntp_peer_data_t ntp_parse_peer_data(const std::string& e)
{
    ntp_peer_data_t ret;

    lines_t params;
    split(e, params);

    unsigned i=1;
    for(lines_t::const_iterator it=params.begin(); it!=params.end(); ++it, i++)
    {
        const std::string& term = *it;

        size_t eq = term.find_first_of('=');
        if(eq==term.npos) {
            if(devntpParseErrors>0)
                errlogPrintf("expected '=' in term %u \"%s\"\n", i, term.c_str());
            continue;
        }

        try {
            ret[strip(term.substr(0, eq))] = strip(term.substr(eq+1));
        } catch(std::runtime_error& e) {
            if(devntpParseErrors>0)
                errlogPrintf("Can't store term %u \"%s\"\n", i, term.c_str());
            continue;
        }
    }

    return ret;
}

void NTPAssembler::add(size_t offset, size_t len, bool more, const void *rawb)
{
    const unsigned char *b = (const unsigned char*)rawb;

    // max. length see so far
    size_t maxlen = std::max(offset+len, buffer.size());

    buffer.resize(maxlen);
    buffer_mask.resize(maxlen, false);

    seen_last |= !more;

    std::copy(b,
              b+len,
              buffer.begin()+offset);
    std::fill_n(buffer_mask.begin()+offset,
                len,
                true);
}

bool NTPAssembler::done() const
{
    // done if last fragment seen and buffer_mask all true
    return seen_last && std::find(buffer_mask.begin(),
                                  buffer_mask.end(),
                                  false)==buffer_mask.end();
}
