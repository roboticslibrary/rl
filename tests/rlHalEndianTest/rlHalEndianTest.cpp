//
// Copyright (c) 2013, Andre Gaschler
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <cassert>
#include <rl/hal/endian.h>

int
main(int argc, char** argv)
{
	{
		uint16_t word = 0x42EC;
		assert(rl::hal::hostEndianWord(rl::hal::highByteFromHostEndian(word), rl::hal::lowByteFromHostEndian(word)) == word);
		assert(rl::hal::littleEndianWord(rl::hal::highByteFromLittleEndian(word), rl::hal::lowByteFromLittleEndian(word)) == word);
		assert(rl::hal::bigEndianWord(rl::hal::highByteFromBigEndian(word), rl::hal::lowByteFromBigEndian(word)) == word);
	}
	{
		uint16_t wordBefore = 0x42EC;
		uint16_t word = wordBefore;
		rl::hal::swapByteOrder(word);
		rl::hal::swapByteOrder(word);
		assert(word == wordBefore);
	}
	{
		uint32_t doubleWord = 0x42EC7654;
		assert(rl::hal::hostEndianDoubleWord(rl::hal::highWordFromHostEndian(doubleWord), rl::hal::lowWordFromHostEndian(doubleWord)) == doubleWord);
		assert(rl::hal::littleEndianDoubleWord(rl::hal::highWordFromLittleEndian(doubleWord), rl::hal::lowWordFromLittleEndian(doubleWord)) == doubleWord);
		assert(rl::hal::bigEndianDoubleWord(rl::hal::highWordFromBigEndian(doubleWord), rl::hal::lowWordFromBigEndian(doubleWord)) == doubleWord);
	}
	return 0;
}
