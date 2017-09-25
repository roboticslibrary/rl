#ifndef RL_SG_SOLID_BODY_H
#define RL_SG_SOLID_BODY_H

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

#endif // RL_SG_SOLID_BODY_H
