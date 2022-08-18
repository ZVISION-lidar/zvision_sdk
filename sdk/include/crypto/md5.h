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

#pragma once

#include <string>
#include <fstream>

/* Type define */
typedef unsigned char byte;
//typedef unsigned int uint32;

//using std::string;
//using std::ifstream;

/* MD5 declaration. */
class CRYPTO_MD5 {
public:
	CRYPTO_MD5();
	CRYPTO_MD5(const void* input, size_t length);
	CRYPTO_MD5(const std::string& str);
	CRYPTO_MD5(std::ifstream& in);
	void update(const void* input, size_t length);
	void update(const std::string& str);
	void update(std::ifstream& in);
	const byte* digest();
	std::string toString();
	void reset();

private:
	void update(const byte* input, size_t length);
	void final();
	void transform(const byte block[64]);
	void encode(const uint32_t* input, byte* output, size_t length);
	void decode(const byte* input, uint32_t* output, size_t length);
	std::string bytesToHexString(const byte* input, size_t length);

	/* class uncopyable */
	CRYPTO_MD5(const CRYPTO_MD5&);
	CRYPTO_MD5& operator=(const CRYPTO_MD5&);

private:
	uint32_t _state[4]; /* state (ABCD) */
	uint32_t _count[2]; /* number of bits, modulo 2^64 (low-order word first) */
	byte _buffer[64]; /* input buffer */
	byte _digest[16]; /* message digest */
	bool _finished;   /* calculate finished ? */

	static const byte PADDING[64]; /* padding for calculate */
	static const char HEX[16];
	enum { BUFFER_SIZE = 1024 };
};
