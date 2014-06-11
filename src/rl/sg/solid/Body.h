#ifndef _RL_SG_SOLID_BODY_H_
#define _RL_SG_SOLID_BODY_H_

#include "../Body.h"

namespace rl
{
	namespace sg
	{
		namespace solid
		{
			class Model;
			
			class Body : public ::rl::sg::Body
			{
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				
				Body(Model* model);
				
				virtual ~Body();
				
				::rl::sg::Shape* create(SoVRMLShape* shape);
				
				void getFrame(::rl::math::Transform& frame);
				
				void setFrame(const ::rl::math::Transform& frame);
				
				void setMargin(const ::rl::math::Real& margin);
				
				::rl::math::Transform frame;
				
			protected:
				
			private:
				
			};
		}
	}
}

#endif // _RL_SG_SOLID_BODY_H_
