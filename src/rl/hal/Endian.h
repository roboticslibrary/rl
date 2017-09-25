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

#ifndef RL_HAL_ENDIAN_H
#define RL_HAL_ENDIAN_H

#include <cstdint>

namespace rl
{
	namespace hal
	{
		class Endian
		{
		public:
			static ::std::uint32_t bigDoubleWord(const ::std::uint16_t& highWord, const ::std::uint16_t& lowWord);
			
			static ::std::uint8_t bigHighByte(const ::std::uint16_t& word);
			
			static ::std::uint32_t bigHighDoubleWord(const ::std::uint64_t& quadWord);
			
			static ::std::uint16_t bigHighWord(const ::std::uint32_t& doubleWord);
			
			static ::std::uint8_t bigLowByte(const ::std::uint16_t& word);
			
			static ::std::uint32_t bigLowDoubleWord(const ::std::uint64_t& quadWord);
			
			static ::std::uint16_t bigLowWord(const ::std::uint32_t& doubleWord);
			
			static ::std::uint64_t bigQuadWord(const ::std::uint32_t& highDoubleWord, const ::std::uint32_t& lowDoubleWord);
			
			static void bigToHost(::std::int8_t& character)
			{
			}
			
			static void bigToHost(::std::uint8_t& character)
			{
			}
			
			static void bigToHost(::std::int16_t& word);
			
			static void bigToHost(::std::uint16_t& word);
			
			static void bigToHost(::std::int32_t& doubleWord);
			
			static void bigToHost(::std::uint32_t& doubleWord);
			
			static void bigToHost(float& real32);
			
			static void bigToHost(::std::int64_t& quadWord);
			
			static void bigToHost(::std::uint64_t& quadWord);
			
			static void bigToHost(double& real64);
			
			static ::std::uint16_t bigWord(const ::std::uint8_t& highByte, const ::std::uint8_t& lowByte);
			
			static ::std::uint32_t hostDoubleWord(const ::std::uint16_t& highWord, const ::std::uint16_t& lowWord)
			{
				return (highWord << 16) | lowWord;
			}
			
			static ::std::uint8_t hostHighByte(const ::std::uint16_t& word)
			{
				return (word >> 8);
			}
			
			static ::std::uint16_t hostHighWord(const ::std::uint32_t& doubleWord)
			{
				return (doubleWord >> 16);
			}
			
			static ::std::uint32_t hostHighDoubleWord(const ::std::uint64_t& quadWord)
			{
				return (quadWord >> 32);
			}
			
			static ::std::uint8_t hostLowByte(const ::std::uint16_t& word)
			{
				return (word & 0xFF);
			}
			
			static ::std::uint16_t hostLowWord(const ::std::uint32_t& doubleWord)
			{
				return (doubleWord & 0xFFFF);
			}
			
			static ::std::uint32_t hostLowDoubleWord(const ::std::uint64_t& quadWord)
			{
				return (quadWord & 0xFFFFFFFF);
			}
			
			static ::std::uint64_t hostQuadWord(const ::std::uint32_t& highDoubleWord, const ::std::uint32_t& lowDoubleWord)
			{
				return (static_cast< ::std::uint64_t>(highDoubleWord) << 32) | lowDoubleWord;
			}
			
			static void hostToBig(::std::int8_t& character)
			{
			}
			
			static void hostToBig(::std::uint8_t& character)
			{
			}
			
			static void hostToBig(::std::int16_t& word);
			
			static void hostToBig(::std::uint16_t& word);
			
			static void hostToBig(::std::int32_t& doubleWord);
			
			static void hostToBig(::std::uint32_t& doubleWord);
			
			static void hostToBig(float& real32);
			
			static void hostToBig(::std::int64_t& quadWord);
			
			static void hostToBig(::std::uint64_t& quadWord);
			
			static void hostToBig(double& real64);
			
			static void hostToLittle(::std::int8_t& character)
			{
			}
			
			static void hostToLittle(::std::uint8_t& character)
			{
			}
			
			static void hostToLittle(::std::int16_t& word);
			
			static void hostToLittle(::std::uint16_t& word);
			
			static void hostToLittle(::std::int32_t& doubleWord);
			
			static void hostToLittle(::std::uint32_t& doubleWord);
			
			static void hostToLittle(float& real32);
			
			static void hostToLittle(::std::int64_t& quadWord);
			
			static void hostToLittle(::std::uint64_t& quadWord);
			
			static void hostToLittle(double& real64);
			
			static ::std::uint16_t hostWord(const ::std::uint8_t& highByte, const ::std::uint8_t& lowByte)
			{
				return (highByte << 8) | lowByte;
			}
			
			static ::std::uint32_t littleDoubleWord(const ::std::uint16_t& highWord, const ::std::uint16_t& lowWord);
			
			static ::std::uint8_t littleHighByte(const ::std::uint16_t& word);
			
			static ::std::uint32_t littleHighDoubleWord(const ::std::uint64_t& quadWord);
			
			static ::std::uint16_t littleHighWord(const ::std::uint32_t& doubleWord);
			
			static ::std::uint8_t littleLowByte(const ::std::uint16_t& word);
			
			static ::std::uint32_t littleLowDoubleWord(const ::std::uint64_t& quadWord);
			
			static ::std::uint16_t littleLowWord(const ::std::uint32_t& doubleWord);
			
			static ::std::uint64_t littleQuadWord(const ::std::uint32_t& highDoubleWord, const ::std::uint32_t& lowDoubleWord);
			
			static void littleToHost(::std::int8_t& character)
			{
			}
			
			static void littleToHost(::std::uint8_t& character)
			{
			}
			
			static void littleToHost(::std::int16_t& word);
			
			static void littleToHost(::std::uint16_t& word);
			
			static void littleToHost(::std::int32_t& doubleWord);
			
			static void littleToHost(::std::uint32_t& doubleWord);
			
			static void littleToHost(float& real32);
			
			static void littleToHost(::std::int64_t& quadWord);
			
			static void littleToHost(::std::uint64_t& quadWord);
			
			static void littleToHost(double& real64);
			
			static ::std::uint16_t littleWord(const ::std::uint8_t& highByte, const ::std::uint8_t& lowByte);
			
			static void reverse(::std::int8_t& character)
			{
			}
			
			static void reverse(::std::uint8_t& character)
			{
			}
			
			static void reverse(::std::int16_t& word)
			{
				word = (word >> 8) | (word << 8);
			}
			
			static void reverse(::std::uint16_t& word)
			{
				word = (word >> 8) | (word << 8);
			}
			
			static void reverse(::std::int32_t& doubleWord)
			{
				doubleWord = ((doubleWord & 0x000000FF) << 24) | ((doubleWord & 0x0000FF00) << 8) |
				             ((doubleWord & 0xFF000000) >> 24) | ((doubleWord & 0x00FF0000) >> 8);
			}
			
			static void reverse(::std::uint32_t& doubleWord)
			{
				doubleWord = ((doubleWord & 0x000000FF) << 24) | ((doubleWord & 0x0000FF00) << 8) |
				             ((doubleWord & 0xFF000000) >> 24) | ((doubleWord & 0x00FF0000) >> 8);
			}
			
			static void reverse(float& real32)
			{
				reverse(*reinterpret_cast< ::std::uint32_t*>(&real32));
			}
			
			static void reverse(::std::int64_t& quadWord)
			{
				quadWord = ((quadWord & 0x00000000000000FF) << 56) | ((quadWord & 0x000000000000FF00) << 40) | ((quadWord & 0x0000000000FF0000) << 24) | ((quadWord & 0x00000000FF000000) << 8) |
				           ((quadWord & 0xFF00000000000000) >> 56) | ((quadWord & 0x00FF000000000000) >> 40) | ((quadWord & 0x0000FF0000000000) >> 24) | ((quadWord & 0x000000FF00000000) >> 8);
			}
			
			static void reverse(::std::uint64_t& quadWord)
			{
				quadWord = ((quadWord & 0x00000000000000FF) << 56) | ((quadWord & 0x000000000000FF00) << 40) | ((quadWord & 0x0000000000FF0000) << 24) | ((quadWord & 0x00000000FF000000) << 8) |
				           ((quadWord & 0xFF00000000000000) >> 56) | ((quadWord & 0x00FF000000000000) >> 40) | ((quadWord & 0x0000FF0000000000) >> 24) | ((quadWord & 0x000000FF00000000) >> 8);
			}
			
			static void reverse(double& real64)
			{
				reverse(*reinterpret_cast< ::std::uint64_t*>(&real64));
			}
			
		protected:
			
		private:
			Endian()
			{
			}
			
			virtual ~Endian()
			{
			}
		};
	}
}

#endif // RL_HAL_ENDIAN_H
