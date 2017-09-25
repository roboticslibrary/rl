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

#include "Endian.h"

#if (defined(HAVE_BIG_ENDIAN) && defined(HAVE_LITTLE_ENDIAN)) || (!defined(HAVE_BIG_ENDIAN) && !defined(HAVE_LITTLE_ENDIAN))
#error define either HAVE_BIG_ENDIAN or HAVE_LITTLE_ENDIAN
#endif // (defined(HAVE_BIG_ENDIAN) && defined(HAVE_LITTLE_ENDIAN)) || (!defined(HAVE_BIG_ENDIAN) && !defined(HAVE_LITTLE_ENDIAN))

namespace rl
{
	namespace hal
	{
		::std::uint32_t
		Endian::bigDoubleWord(const ::std::uint16_t& highWord, const ::std::uint16_t& lowWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostDoubleWord(highWord, lowWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return (lowWord << 16) | highWord;
#endif
		}
		
		::std::uint8_t
		Endian::bigHighByte(const ::std::uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostHighByte(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostLowByte(word);
#endif
		}
		
		::std::uint32_t
		Endian::bigHighDoubleWord(const ::std::uint64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostHighDoubleWord(quadWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostLowDoubleWord(quadWord);
#endif
		}
		
		::std::uint16_t
		Endian::bigHighWord(const ::std::uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostHighWord(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostLowWord(doubleWord);
#endif
		}
		
		::std::uint8_t
		Endian::bigLowByte(const ::std::uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostLowByte(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostHighByte(word);
#endif
		}
		
		::std::uint32_t
		Endian::bigLowDoubleWord(const ::std::uint64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostLowDoubleWord(quadWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostHighDoubleWord(quadWord);
#endif
		}
		
		::std::uint16_t
		Endian::bigLowWord(const ::std::uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostLowWord(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostHighWord(doubleWord);
#endif
		}
		
		::std::uint64_t
		Endian::bigQuadWord(const ::std::uint32_t& highDoubleWord, const ::std::uint32_t& lowDoubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostQuadWord(highDoubleWord, lowDoubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return (static_cast< ::std::uint64_t>(lowDoubleWord) << 32) | highDoubleWord;
#endif
		}
		
		void
		Endian::bigToHost(::std::int16_t& word)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(word);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::bigToHost(::std::uint16_t& word)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(word);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::bigToHost(::std::int32_t& doubleWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(doubleWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::bigToHost(::std::uint32_t& doubleWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(doubleWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::bigToHost(float& real32)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(real32);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::bigToHost(::std::int64_t& quadWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(quadWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::bigToHost(::std::uint64_t& quadWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(quadWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::bigToHost(double& real64)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(real64);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		::std::uint16_t
		Endian::bigWord(const ::std::uint8_t& highByte, const ::std::uint8_t& lowByte)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostWord(highByte, lowByte);
#elif defined(HAVE_LITTLE_ENDIAN)
			return (lowByte << 8) | highByte;
#endif
		}
		
		void
		Endian::hostToBig(::std::int16_t& word)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(word);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToBig(::std::uint16_t& word)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(word);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToBig(::std::int32_t& doubleWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(doubleWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToBig(::std::uint32_t& doubleWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(doubleWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToBig(float& real32)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(real32);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToBig(::std::int64_t& quadWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(quadWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToBig(::std::uint64_t& quadWord)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(quadWord);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToBig(double& real64)
		{
#ifdef HAVE_LITTLE_ENDIAN
			reverse(real64);
#endif // HAVE_LITTLE_ENDIAN
		}
		
		void
		Endian::hostToLittle(::std::int16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(word);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::hostToLittle(::std::uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(word);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::hostToLittle(::std::int32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(doubleWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::hostToLittle(::std::uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(doubleWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::hostToLittle(float& real32)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(doubleWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::hostToLittle(::std::int64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(quadWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::hostToLittle(::std::uint64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(quadWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::hostToLittle(double& real64)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(quadWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		::std::uint32_t
		Endian::littleDoubleWord(const ::std::uint16_t& highWord, const ::std::uint16_t& lowWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return (lowWord << 16) | highWord;
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostDoubleWord(highWord, lowWord);
#endif
		}
		
		::std::uint8_t
		Endian::littleHighByte(const ::std::uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostLowByte(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostHighByte(word);
#endif
		}
		
		::std::uint32_t
		Endian::littleHighDoubleWord(const ::std::uint64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostLowDoubleWord(quadWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostHighDoubleWord(quadWord);
#endif
		}
		
		::std::uint16_t
		Endian::littleHighWord(const ::std::uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostLowWord(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostHighWord(doubleWord);
#endif
		}
		
		::std::uint8_t
		Endian::littleLowByte(const ::std::uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostHighByte(word);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostLowByte(word);
#endif
		}
		
		::std::uint32_t
		Endian::littleLowDoubleWord(const ::std::uint64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostHighDoubleWord(quadWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostLowDoubleWord(quadWord);
#endif
		}
		
		::std::uint16_t
		Endian::littleLowWord(const ::std::uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return hostHighWord(doubleWord);
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostLowWord(doubleWord);
#endif
		}
		
		::std::uint64_t
		Endian::littleQuadWord(const ::std::uint32_t& highDoubleWord, const ::std::uint32_t& lowDoubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			return (static_cast< ::std::uint64_t>(lowDoubleWord) << 32) | highDoubleWord;
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostQuadWord(highDoubleWord, lowDoubleWord);
#endif
		}
		
		void
		Endian::littleToHost(::std::int16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(word);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::littleToHost(::std::uint16_t& word)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(word);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::littleToHost(::std::int32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(doubleWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::littleToHost(::std::uint32_t& doubleWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(doubleWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::littleToHost(float& real32)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(real32);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::littleToHost(::std::int64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(quadWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::littleToHost(::std::uint64_t& quadWord)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(quadWord);
#endif // HAVE_BIG_ENDIAN
		}
		
		void
		Endian::littleToHost(double& real64)
		{
#ifdef HAVE_BIG_ENDIAN
			reverse(real64);
#endif // HAVE_BIG_ENDIAN
		}
		
		::std::uint16_t
		Endian::littleWord(const ::std::uint8_t& highByte, const ::std::uint8_t& lowByte)
		{
#ifdef HAVE_BIG_ENDIAN
			return (lowByte << 8) | highByte;
#elif defined(HAVE_LITTLE_ENDIAN)
			return hostWord(highByte, lowByte);
#endif
		}
	}
}
