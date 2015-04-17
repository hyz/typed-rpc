#include <boost/algorithm/string.hpp>
#include <boost/random.hpp>
//#include <boost/filesystem/path.hpp>
//#include <boost/filesystem/fstream.hpp>
//#include <boost/filesystem/operations.hpp>
//#include <boost/lexical_cast.hpp>
#include <algorithm>
#include "util.h"

using namespace std;
using namespace boost;

string path_leaf(const string& path)
{
    // typedef boost::range_reverse_iterator<string>::type rev_iterator;
    string::const_reverse_iterator i = std::find(path.rbegin(), path.rend(), '/');
    if (i != path.rend())
    {
        return string(i.base(), path.end());
    }
    return path;
}


/* {{{ php_htoi
 */
static int php_htoi(char *s)
{
	int value;
	int c;

	c = ((unsigned char *)s)[0];
	if (isupper(c))
		c = tolower(c);
	value = (c >= '0' && c <= '9' ? c - '0' : c - 'a' + 10) * 16;

	c = ((unsigned char *)s)[1];
	if (isupper(c))
		c = tolower(c);
	value += c >= '0' && c <= '9' ? c - '0' : c - 'a' + 10;

	return (value);
}
/* }}} */

/* {{{ URLè§£ç ï¼æåèªPHP 5.2.17
   ç¨æ³ï¼string urldecode(string str_source)
   æ¶é´ï¼2012-8-14 By Dewei
*/
string urldecode(const char *in_str, const char *end)
{
    int in_str_len = end - in_str;
	// int out_str_len = 0;
	string out_str;
	char *str;

	str = strdup(in_str);
	char *dest = str;
	char *data = str;

	while (in_str_len--) {
		if (*data == '+') {
			*dest = ' ';
		}
		else if (*data == '%' && in_str_len >= 2 && isxdigit((int) *(data + 1)) 
			&& isxdigit((int) *(data + 2))) {
				*dest = (char) php_htoi(data + 1);
				data += 2;
				in_str_len -= 2;
		} else {
			*dest = *data;
		}
		data++;
		dest++;
	}
	*dest = '\0';
	// out_str_len =  dest - str;
	out_str = str;
	free(str);
    return out_str;
}
string urldecode(const string &str_source)
{
	char const *in_str = str_source.c_str();
	//int in_str_len = strlen(in_str);
    return urldecode(in_str, in_str + str_source.size());
}

/* }}} */


/* {{{ URLç¼ç ï¼æåèªPHP 
   ç¨æ³ï¼string urlencode(string str_source)
   è¯´æï¼ä»ä¸ç¼ç  -_. å¶ä½å¨é¨ç¼ç ï¼ç©ºæ ¼ä¼è¢«ç¼ç ä¸º +
   æ¶é´ï¼2012-8-13 By Dewei
*/
string urlencode(const string &str_source)
{
	char const *in_str = str_source.c_str();
	int in_str_len = strlen(in_str);
	// int out_str_len = 0;
	string out_str;
	register unsigned char c;
	unsigned char *to, *start;
	unsigned char const *from, *end;
	unsigned char hexchars[] = "0123456789ABCDEF";

	from = (unsigned char *)in_str;
	end = (unsigned char *)in_str + in_str_len;
	start = to = (unsigned char *) malloc(3*in_str_len+1);

	while (from < end) {
		c = *from++;

		if (c == ' ') {
			*to++ = '+';
		} else if ((c < '0' && c != '-' && c != '.') ||
			(c < 'A' && c > '9') ||
			(c > 'Z' && c < 'a' && c != '_') ||
			(c > 'z')) { 
				to[0] = '%';
				to[1] = hexchars[c >> 4];
				to[2] = hexchars[c & 15];
				to += 3;
		} else {
			*to++ = c;
		}
	}
	*to = 0;

	// out_str_len = to - start;
	out_str = (char *) start;
	free(start);
	return out_str;
}
/* }}} */

std::string UrlEncode(const std::string& szToEncode)
{
	std::string src = szToEncode;
	char hex[] = "0123456789ABCDEF";
	string dst;

	for (size_t i = 0; i < src.size(); ++i)
	{
		unsigned char cc = src[i];
		if (isascii(cc))
		{
			if (cc == ' ')
			{
				dst += "%20";
			}
			else
				dst += cc;
		}
		else
		{
			unsigned char c = static_cast<unsigned char>(src[i]);
			dst += '%';
			dst += hex[c / 16];
			dst += hex[c % 16];
		}
	}
	return dst;
}


std::string UrlDecode(const std::string& szToDecode)
{
	std::string result;
	int hex = 0;
	for (size_t i = 0; i < szToDecode.length(); ++i)
	{
		switch (szToDecode[i])
		{
		case '+':
			result += ' ';
			break;
		case '%':
			if (isxdigit(szToDecode[i + 1]) && isxdigit(szToDecode[i + 2]))
			{
				std::string hexStr = szToDecode.substr(i + 1, 2);
				hex = strtol(hexStr.c_str(), 0, 16);
				// å­æ¯åæ°å­[0-9a-zA-Z]ãä¸äºç¹æ®ç¬¦å·[$-_.+!*'(),] ãä»¥åæäºä¿çå­[$&+,/:;=?@]
				// å¯ä»¥ä¸ç»è¿ç¼ç ç´æ¥ç¨äºURL
				if (!((hex >= 48 && hex <= 57) ||	//0-9
					(hex >=97 && hex <= 122) ||	//a-z
					(hex >=65 && hex <= 90) ||	//A-Z
					// ä¸äºç¹æ®ç¬¦å·åä¿çå­[$-_.+!*'(),]  [$&+,/:;=?@]
					hex == 0x21 || hex == 0x24 || hex == 0x26 || hex == 0x27 || hex == 0x28 || hex == 0x29
					|| hex == 0x2a || hex == 0x2b|| hex == 0x2c || hex == 0x2d || hex == 0x2e || hex == 0x2f
					|| hex == 0x3A || hex == 0x3B|| hex == 0x3D || hex == 0x3f || hex == 0x40 || hex == 0x5f
					))
				{
					result += char(hex);
					i += 2;
				}
				else result += '%';
			}else {
				result += '%';
			}
			break;
		default:
			result += szToDecode[i];
			break;
		}
	}
	return result;
}

unsigned int randuint(unsigned int beg, unsigned int end)
{
    static boost::mt19937 gen_(time(0));
    boost::random::uniform_int_distribution<unsigned int> dist(beg,end); //(100000,999999);
    unsigned int ret = dist(gen_);
    return ret;
}

std::string readfile(boost::filesystem::path const & fp)
{
    boost::filesystem::ifstream inf(fp);
    if (!inf)
        return std::string();
    std::istreambuf_iterator<char> i(inf), end;
    return std::string(i, end);
}

void copyfile(std::string const & src, std::string const & dst)
{
    boost::filesystem::ifstream inf(src);
    boost::filesystem::ofstream outf(dst);
    if (!inf || !outf)
        return;
    outf << inf.rdbuf();
}

inline boost::filesystem::path mk_filename(boost::filesystem::path const& dir, struct tm const* tm)
{
    return (dir / boost::lexical_cast<std::string>(tm->tm_wday));
}

