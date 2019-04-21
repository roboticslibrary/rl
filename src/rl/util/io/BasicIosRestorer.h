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

#ifndef RL_UTIL_IO_BASICIOSRESTORER_H
#define RL_UTIL_IO_BASICIOSRESTORER_H

#include <ios>

namespace rl
{
	namespace util
	{
		namespace io
		{
			template<typename CharT, typename Traits = ::std::char_traits<CharT>>
			class BasicIosRestorer
			{
			public:
				typedef ::std::basic_ios<CharT, Traits> value_type;
				
				explicit BasicIosRestorer(value_type& ios) :
					exceptions(ios.exceptions()),
					fill(ios.fill()),
					flags(ios.flags()),
					ios(ios),
					precision(ios.precision()),
					rdbuf(ios.rdbuf()),
					rdstate(ios.rdstate()),
					tie(ios.tie()),
					width(ios.width())
				{
				}
				
				~BasicIosRestorer()
				{
					this->ios.exceptions(this->exceptions);
					this->ios.fill(this->fill);
					this->ios.flags(this->flags);
					this->ios.precision(this->precision);
					this->ios.rdbuf(this->rdbuf);
					this->ios.clear(this->rdstate);
					this->ios.tie(this->tie);
					this->ios.width(this->width);
				}
				
			protected:
				
			private:
				const typename value_type::iostate exceptions;
				
				const typename value_type::char_type fill;
				
				const typename value_type::fmtflags flags;
				
				value_type& ios;
				
				const typename ::std::streamsize precision;
				
				::std::basic_streambuf<CharT, Traits>* rdbuf;
				
				const typename value_type::iostate rdstate;
				
				::std::basic_ostream<CharT, Traits>* tie;
				
				const typename ::std::streamsize width;
			};
		}
	}
}

#endif // RL_UTIL_IO_BASICIOSRESTORER_H
