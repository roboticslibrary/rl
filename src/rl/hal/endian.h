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

#ifndef _RL_HAL_ENDIAN_H_
#define _RL_HAL_ENDIAN_H_

#include "types.h"

namespace rl
{
	namespace hal
	{
		uint32_t bigEndianDoubleWord(const uint16_t& highWord, const uint16_t& lowWord);
		
		void bigEndianToHostEndian(uint16_t& word);
		
		void bigEndianToHostEndian(uint32_t& doubleWord);
		
		uint16_t bigEndianWord(const uint8_t& highByte, const uint8_t& lowByte);
		
		uint8_t highByteFromBigEndian(const uint16_t& word);
		
		inline uint8_t highByteFromHostEndian(const uint16_t& word)
		{
			return (word >> 8);
		}
		
		uint8_t highByteFromLittleEndian(const uint16_t& word);
		
		uint16_t highWordFromBigEndian(const uint32_t& doubleWord);
		
		inline uint16_t highWordFromHostEndian(const uint32_t& doubleWord)
		{
			return (doubleWord >> 16);
		}
		
		uint16_t highWordFromLittleEndian(const uint32_t& doubleWord);
		
		inline uint32_t hostEndianDoubleWord(const uint16_t& highWord, const uint16_t& lowWord)
		{
			return (highWord << 16) | lowWord;
		}
		
		void hostEndianToBigEndian(uint16_t& word);
		
		void hostEndianToBigEndian(uint32_t& doubleWord);
		
		void hostEndianToLittleEndian(uint16_t& word);
		
		void hostEndianToLittleEndian(uint32_t& doubleWord);
		
		inline uint16_t hostEndianWord(const uint8_t& highByte, const uint8_t& lowByte)
		{
			return (highByte << 8) | lowByte;
		}
		
		uint32_t littleEndianDoubleWord(const uint16_t& highWord, const uint16_t& lowWord);
		
		void littleEndianToHostEndian(uint16_t& word);
		
		void littleEndianToHostEndian(uint32_t& doubleWord);
		
		uint16_t littleEndianWord(const uint8_t& highByte, const uint8_t& lowByte);
		
		uint8_t lowByteFromBigEndian(const uint16_t& word);
		
		inline uint8_t lowByteFromHostEndian(const uint16_t& word)
		{
			return (word & 0xFF);
		}
		
		uint8_t lowByteFromLittleEndian(const uint16_t& word);
		
		uint16_t lowWordFromBigEndian(const uint32_t& doubleWord);
		
		inline uint16_t lowWordFromHostEndian(const uint32_t& doubleWord)
		{
			return (doubleWord & 0xFFFF);
		}
		
		uint16_t lowWordFromLittleEndian(const uint32_t& doubleWord);
		
		inline void swapByteOrder(uint16_t& word)
		{
			word = (word >> 8) | (word << 8); // TODO
		}
		
		inline void swapByteOrder(uint32_t& doubleWord)
		{
			doubleWord = ((doubleWord & 0x000000FF) << 24) | ((doubleWord & 0x0000FF00) << 8) | ((doubleWord & 0x00FF0000) >> 8) | ((doubleWord & 0xFF000000) >> 24); // TODO
		}
	}
}

#endif // _RL_HAL_ENDIAN_H_
