// MIT License
//
// Copyright(c) 2019 ZVISION. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <map>
#include <unordered_map>
#include <string.h>
#include <math.h>
#include "print.h"
#ifdef WIN32
#include<Windows.h>
#include<io.h>
#else
#include<dirent.h>
#endif

class ParamResolver
{
public:

    static int GetParameters(int argc, char* argv[], std::map<std::string, std::string>& paras, std::string& appname)
    {
        paras.clear();
        if (argc >= 1)
            appname = std::string(argv[0]);

        std::string key;
        std::string value;
        for (int i = 1; i < argc; i++)
        {
            std::string str(argv[i]);
            if ((str.size() > 1) && ('-' == str[0]))
            {
                key = str;
                if (i == (argc - 1))
                    value = "";
                else
                {
                    value = std::string(argv[i + 1]);
                    if ('-' == value[0])
                    {
                        value = "";
                    }
                    else
                    {
                        i++;
                    }
                }
                paras[key] = value;
            }
        }
        return 0;
    }

};

/* String split */
void strSplit(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters = " ") {
	std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
	std::string::size_type pos = s.find_first_of(delimiters, lastPos);
	while (std::string::npos != pos || std::string::npos != lastPos) {
		tokens.push_back(s.substr(lastPos, pos - lastPos));
		//use emplace_back after C++11
		lastPos = s.find_first_not_of(delimiters, pos);
		pos = s.find_first_of(delimiters, lastPos);
	}
}

/* Verify ip */
bool AssembleIpString(const std::string& ip)
{
	try
	{
		/* ip format: xxx.xxx.xxx.xxx */
		if (ip.size() > 15)
			return false;

		std::vector<std::string> vals;
		strSplit(ip, vals, ".");
		if (vals.size() != 4)
			return false;

		for (auto s : vals) {
			for (auto c : s) {
				if (c<'0' || c>'9')
					return false;
			}
		}
	}
	catch (const std::exception& e)
	{
		return false;
	}
	return true;
}

void getFileListInDir(std::string path, std::vector<std::string>& filelist)
{
#ifdef WIN32
    std::string reg = path + "*.*";
    WIN32_FIND_DATA fdata;
    HANDLE hFirst = FindFirstFile(reg.c_str(), &fdata);
    HANDLE hFind = hFirst;
    if (hFind == INVALID_HANDLE_VALUE)
        return;

    do
    {
        if (strcmp(fdata.cFileName, ".") == 0 || strcmp(fdata.cFileName, "..") == 0)
        {
            FindNextFile(hFind, &fdata);
            continue;
        }

        if (!(fdata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
            filelist.push_back(std::string(fdata.cFileName));

    } while (FindNextFile(hFind, &fdata));
    FindClose(hFirst);

#else
    // open dir
    
    DIR* pdir = NULL;
    if(!(pdir = opendir(path.c_str())))
        return;

    // get file list
    struct dirent* ptr;
    while((ptr = readdir(pdir) )!= 0)
    {
        if(strcmp(ptr->d_name,".") && strcmp(ptr->d_name, ".."))
            filelist.push_back(ptr->d_name);
    }
    closedir(pdir);
#endif
}

void Host2Network(const unsigned char* host, char* net, int len = 4)
{
    for (int i = 0; i < len / 4; i++)
    {
        int ori = *(int*)(host + 4 * i);
        int* now = (int*)(net + 4 * i);
        *now = htonl(ori);
    }
}

