//
// Copyright (c) 2009, Markus Rickert
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

#include "endian.h"

#if (defined(HAVE_BIG_ENDIAN) && defined(HAVE_LITTLE_ENDIAN)) || (!defined(HAVE_BIG_ENDIAN) && !defined(HAVE_LITTLE_ENDIAN))
#error define either HAVE_BIG_ENDIAN or HAVE_LITTLE_ENDIAN
#endif // (defined(HAVE_BIG_ENDIAN) && defined(HAVE_LITTLE_ENDIAN)) || (!defined(HAVE_BIG_ENDIAN) && !defined(HAVE_LITTLE_ENDIAN))

namespace rl
{
	namespace hal
	{
		uint32_t bigEndianDoubleWord(const uint16_t& highWord, const uint16_t& lowWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostEndianDoubleWord(highWord, lowWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return (lowWord << 16) | highWord;
#endif
		}
		
		void bigEndianToHostEndian(uint16_t& word)
		{
#ifdef HAVE_LITTLE_ENDIAN
			swapByteOrder(word);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void bigEndianToHostEndian(uint32_t& doubleWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			swapByteOrder(doubleWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		uint16_t bigEndianWord(const uint8_t& highByte, const uint8_t& lowByte)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostEndianWord(highByte, lowByte);
#elif defined(HAVE_LITTLE_ENDIAN)
			return (lowByte << 8) | highByte;
#endif
		}
		
		uint8_t highByteFromBigEndian(const uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return highByteFromHostEndian(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return lowByteFromHostEndian(word);
#endif
		}
		
		uint8_t highByteFromLittleEndian(const uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return lowByteFromHostEndian(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return highByteFromHostEndian(word);
#endif
		}
		
		uint16_t highWordFromBigEndian(const uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return highWordFromHostEndian(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return lowWordFromHostEndian(doubleWord);
#endif
		}
		
		uint16_t highWordFromLittleEndian(const uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return lowWordFromHostEndian(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return highWordFromHostEndian(doubleWord);
#endif
		}
		
		void hostEndianToBigEndian(uint16_t& word)
		{
#ifdef HAVE_LITTLE_ENDIAN
			swapByteOrder(word);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void hostEndianToBigEndian(uint32_t& doubleWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			swapByteOrder(doubleWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void hostEndianToLittleEndian(uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			swapByteOrder(word);
#endif // HAVE_BIG_ENDIAN
		}
		
		void hostEndianToLittleEndian(uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			swapByteOrder(doubleWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		uint32_t littleEndianDoubleWord(const uint16_t& highWord, const uint16_t& lowWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return (lowWord << 16) | highWord;
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostEndianDoubleWord(highWord, lowWord);
#endif
		}
		
		void littleEndianToHostEndian(uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			swapByteOrder(word);
#endif // HAVE_BIG_ENDIAN
		}
		
		void littleEndianToHostEndian(uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			swapByteOrder(doubleWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		uint16_t littleEndianWord(const uint8_t& highByte, const uint8_t& lowByte)
		{
#ifdef HAVE_BIG_ENDIAN
			return (lowByte << 8) | highByte;
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostEndianWord(highByte, lowByte);
#endif
		}
		
		uint8_t lowByteFromBigEndian(const uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return lowByteFromHostEndian(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return highByteFromHostEndian(word);
#endif
		}
		
		uint8_t lowByteFromLittleEndian(const uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return highByteFromHostEndian(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return lowByteFromHostEndian(word);
#endif
		}
		
		uint16_t lowWordFromBigEndian(const uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return lowWordFromHostEndian(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return highWordFromHostEndian(doubleWord);
#endif
		}
		
		uint16_t lowWordFromLittleEndian(const uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return highWordFromHostEndian(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return lowWordFromHostEndian(doubleWord);
#endif
		}
	}
}
