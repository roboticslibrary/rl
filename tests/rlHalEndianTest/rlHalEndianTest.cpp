//
// Copyright (c) 2013, Andre Gaschler, Markus Rickert
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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <rl/hal/Endian.h>

int
main(int argc, char** argv)
{
	{
		::std::uint16_t wordBefore = 0x42EC;
		
		std::cout << "host   " << std::hex << wordBefore << std::endl;
		
		::std::uint16_t word = rl::hal::Endian::hostWord(
			rl::hal::Endian::hostHighByte(wordBefore),
			rl::hal::Endian::hostLowByte(wordBefore)
		);
		
		std::cout << "host   " << std::hex << word << std::endl;
		
		if (word != wordBefore)
		{
			std::cout << std::hex << word << " != " << wordBefore << std::endl;
			return EXIT_FAILURE;
		}
		
		word = rl::hal::Endian::littleWord(
			rl::hal::Endian::littleHighByte(wordBefore),
			rl::hal::Endian::littleLowByte(wordBefore)
		);
		
		std::cout << "little " << std::hex << word << std::endl;
		
		if (word != wordBefore)
		{
			std::cout << std::hex << word << " != " << wordBefore << std::endl;
			return EXIT_FAILURE;
		}
		
		word = rl::hal::Endian::bigWord(
			rl::hal::Endian::bigHighByte(wordBefore),
			rl::hal::Endian::bigLowByte(wordBefore)
		);
		
		std::cout << "big    " << std::hex << word << std::endl;
		
		if (word != wordBefore)
		{
			std::cout << std::hex << word << " != " << wordBefore << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << std::endl;
	
	{
		::std::uint16_t wordBefore = 0x42EC;
		std::cout << std::hex << "                word   " << wordBefore << std::endl;
		::std::uint16_t word = wordBefore;
		rl::hal::Endian::reverse(word);
		std::cout << std::hex << "        reverse(word)  " << word << std::endl;
		rl::hal::Endian::reverse(word);
		std::cout << std::hex << "reverse(reverse(word)) " << word << std::endl;
		
		if (word != wordBefore)
		{
			std::cout << std::hex << word << " != " << wordBefore << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << std::endl;
	
	{
		::std::uint32_t doubleWordBefore = 0x42EC7654;
		
		std::cout << "host   " << std::hex << doubleWordBefore << std::endl;
		
		::std::uint32_t doubleWord = rl::hal::Endian::hostDoubleWord(
			rl::hal::Endian::hostHighWord(doubleWordBefore),
			rl::hal::Endian::hostLowWord(doubleWordBefore)
		);
		
		std::cout << "host   " << std::hex << doubleWord << std::endl;
		
		if (doubleWord != doubleWordBefore)
		{
			std::cout << std::hex << doubleWord << " != " << doubleWordBefore << std::endl;
			return EXIT_FAILURE;
		}
		
		doubleWord = rl::hal::Endian::littleDoubleWord(
			rl::hal::Endian::littleHighWord(doubleWordBefore),
			rl::hal::Endian::littleLowWord(doubleWordBefore)
		);
		
		std::cout << "little " << std::hex << doubleWord << std::endl;
		
		if (doubleWord != doubleWordBefore)
		{
			std::cout << std::hex << doubleWord << " != " << doubleWordBefore << std::endl;
			return EXIT_FAILURE;
		}
		
		doubleWord = rl::hal::Endian::bigDoubleWord(
			rl::hal::Endian::bigHighWord(doubleWordBefore),
			rl::hal::Endian::bigLowWord(doubleWordBefore)
		);
		
		std::cout << "big    " << std::hex << doubleWord << std::endl;
		
		if (doubleWord != doubleWordBefore)
		{
			std::cout << std::hex << doubleWord << " != " << doubleWordBefore << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << std::endl;
	
	{
		::std::uint32_t doubleWordBefore = 0x42EC7654;
		std::cout << std::hex << "                doubleWord   " << doubleWordBefore << std::endl;
		::std::uint32_t doubleWord = doubleWordBefore;
		rl::hal::Endian::reverse(doubleWord);
		std::cout << std::hex << "        reverse(doubleWord)  " << doubleWord << std::endl;
		rl::hal::Endian::reverse(doubleWord);
		std::cout << std::hex << "reverse(reverse(doubleWord)) " << doubleWord << std::endl;
		
		if (doubleWord != doubleWordBefore)
		{
			std::cout << std::hex << doubleWord << " != " << doubleWordBefore << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << std::endl;
	
	{
		float real32Before = std::sqrt(2.0f);
		std::cout << "                float   " << real32Before << std::endl;
		float real32 = real32Before;
		rl::hal::Endian::reverse(real32);
		std::cout << "        reverse(float)  " << real32 << std::endl;
		rl::hal::Endian::reverse(real32);
		std::cout << "reverse(reverse(float)) " << real32 << std::endl;
		
		if (real32 != real32Before)
		{
			std::cout << std::hex << real32 << " != " << real32Before << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << std::endl;
	
	{
		::std::uint64_t quadWordBefore = 0x42EC7654C5BD91E0;
		
		std::cout << "host   " << std::hex << quadWordBefore << std::endl;
		
		::std::uint64_t quadWord = rl::hal::Endian::hostQuadWord(
			rl::hal::Endian::hostHighDoubleWord(quadWordBefore),
			rl::hal::Endian::hostLowDoubleWord(quadWordBefore)
		);
		
		std::cout << "host   " << std::hex << quadWord << std::endl;
		
		if (quadWord != quadWordBefore)
		{
			std::cout << std::hex << quadWord << " != " << quadWordBefore << std::endl;
			return EXIT_FAILURE;
		}
		
		quadWord = rl::hal::Endian::littleQuadWord(
			rl::hal::Endian::littleHighDoubleWord(quadWordBefore),
			rl::hal::Endian::littleLowDoubleWord(quadWordBefore)
		);
		
		std::cout << "little " << std::hex << quadWord << std::endl;
		
		if (quadWord != quadWordBefore)
		{
			std::cout << std::hex << quadWord << " != " << quadWordBefore << std::endl;
			return EXIT_FAILURE;
		}
		
		quadWord = rl::hal::Endian::bigQuadWord(
			rl::hal::Endian::bigHighDoubleWord(quadWordBefore),
			rl::hal::Endian::bigLowDoubleWord(quadWordBefore)
		);
		
		std::cout << "big    " << std::hex << quadWord << std::endl;
		
		if (quadWord != quadWordBefore)
		{
			std::cout << std::hex << quadWord << " != " << quadWordBefore << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << std::endl;
	
	{
		::std::uint64_t quadWordBefore = 0x42EC76546D8B43AC;
		std::cout << std::hex << "                quadWord   " << quadWordBefore << std::endl;
		::std::uint64_t quadWord = quadWordBefore;
		rl::hal::Endian::reverse(quadWord);
		std::cout << std::hex << "        reverse(quadWord)  " << quadWord << std::endl;
		rl::hal::Endian::reverse(quadWord);
		std::cout << std::hex << "reverse(reverse(quadWord)) " << quadWord << std::endl;
		
		if (quadWord != quadWordBefore)
		{
			std::cout << std::hex << quadWord << " != " << quadWordBefore << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	std::cout << std::endl;
	
	{
		double real64Before = std::sqrt(2.0);
		std::cout << std::hex << "                double   " << real64Before << std::endl;
		double real64 = real64Before;
		rl::hal::Endian::reverse(real64);
		std::cout << std::hex << "        reverse(double)  " << real64 << std::endl;
		rl::hal::Endian::reverse(real64);
		std::cout << std::hex << "reverse(reverse(double)) " << real64 << std::endl;
		
		if (real64 != real64Before)
		{
			std::cout << std::hex << real64 << " != " << real64Before << std::endl;
			return EXIT_FAILURE;
		}
	}
	
	return EXIT_SUCCESS;
}
