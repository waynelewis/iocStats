
#include <sstream>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "ntphelper.h"

namespace {

template<typename A, typename B>
void testEqualx(const A& a, const B& b, const char *sa, const char *sb)
{
    std::ostringstream strm;
    strm<<sa<<" ("<<a<<") == "<<sb<<" ("<<b<<")";
    testOk(a==b, "%s", strm.str().c_str());
}
#define testEqual(A,B) testEqualx(A, B, #A, #B)

void testEntryEqual(const ntp_data_t& values, const char *key, const char *value)
{
    ntp_data_t::const_iterator it=values.find(key);
    if(it==values.end())
        testFail("missing key %s", key);
    else
        testOk(it->second==value, "values[%s] == %s (%s)", key, value, it->second.c_str());
}

void testParse()
{
    testDiag("In %s", __FUNCTION__);
    const char input[] = "srcadr=107.170.224.8, srcport=123, dstadr=35.22.118.195, dstport=123,\r\n"
                         "leap=0, stratum=2, precision=-22, rootdelay=29.358, rootdisp=57.571,\r\n"
                         "refid=132.163.4.102, reftime=0xdbcf1959.66bb35d4,\r\n"
                         "rec=0xdbcf1c41.66ea524b, reach=0x7, unreach=0, hmode=3, pmode=4,\r\n"
                         "hpoll=10, ppoll=10, headway=0, flash=0x0, keyid=0, offset=-1.265,\r\n"
                         "delay=62.072, dispersion=1.938, jitter=34.338, xleave=0.046,\r\n"
                         "filtdelay= 64.32 244.01 62.38 62.07 62.66 62.82 62.18 62.65,\r\n"
                         "filtoffset= -1.33 -92.12 -1.40 -1.27 -1.46 -1.25 -1.18 -1.44,\r\n"
                         "filtdisp= 0.00 1.04 1.91 1.94 1.97 2.00 2.03 2.06\r\n";

    ntp_data_t values(ntp_parse_peer_data(input));

    testEqual(values.size(), 22u);
    testEntryEqual(values, "srcadr", "107.170.224.8");
    testEntryEqual(values, "rootdelay", "29.358");
    testEntryEqual(values, "rootdisp", "57.571");
    testEntryEqual(values, "filtoffset", "-1.33 -92.12 -1.40 -1.27 -1.46 -1.25 -1.18 -1.44");
}

void testParse2()
{
    testDiag("In %s", __FUNCTION__);
    // throw in some whitespace where it would not normally appear
    const char input[] = "  hello =\tworld ,\t this = a test  ";

    ntp_data_t values(ntp_parse_peer_data(input));

    testEqual(values.size(), 2u);
    testEntryEqual(values, "hello", "world");
    testEntryEqual(values, "this", "a test");
}

void testParseError()
{
    testDiag("In %s", __FUNCTION__);
    // some malformed entries, which should be ignored
    const char input[] = "  hello =\tworld , foo, bar=, =baz, =, ,,\t this = a test  ";

    ntp_data_t values(ntp_parse_peer_data(input));

    testEqual(values.size(), 2u);
    testEntryEqual(values, "hello", "world");
    testEntryEqual(values, "this", "a test");
}

void testAssembleSingle()
{
    testDiag("In %s", __FUNCTION__);
    NTPAssembler A;

    testOk1(!A.done());

    A.add(0, 5, false, "hello");
    testOk1(A.done());

    testEqual(A.tostring(), "hello");
}

void testAssemble2()
{
    testDiag("In %s", __FUNCTION__);
    NTPAssembler A;

    testOk1(!A.done());

    A.add(0, 5, true, "hello");
    testOk1(!A.done());

    A.add(5, 6, false, " world");
    testOk1(A.done());

    testEqual(A.tostring(), "hello world");
}

void testAssemble2Reverse()
{
    testDiag("In %s", __FUNCTION__);
    NTPAssembler A;

    testOk1(!A.done());

    A.add(5, 6, false, " world");
    testOk1(!A.done());

    A.add(0, 5, true, "hello");
    testOk1(A.done());

    testEqual(A.tostring(), "hello world");
}

void testAssemble3()
{
    testDiag("In %s", __FUNCTION__);
    NTPAssembler A;

    testOk1(!A.done());

    A.add(0, 5, true, "hello");
    testOk1(!A.done());

    A.add(6, 5, false, "world");
    testOk1(!A.done());

    A.add(5, 1, true, " ");
    testOk1(A.done());

    testEqual(A.tostring(), "hello world");
}

} // namespace

MAIN(testntphelper)
{
    testPlan(27);
    try {
        testParse();
        testParse2();
        testParseError();

        testAssembleSingle();
        testAssemble2();
        testAssemble2Reverse();
        testAssemble3();
    }catch(std::exception& e){
        testAbort("Unexpected exception: %s", e.what());
    }
    return testDone();
}
