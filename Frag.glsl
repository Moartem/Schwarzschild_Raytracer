#version 330

	//in vec4 v_Color;
	out vec4 out_Color;
	in vec2 v_screenPosition;

	uniform mat3 first;
	uniform mat3 second;
	uniform mat3 third;
	uniform float beta = 0.01;
	uniform sampler2D tex;
	uniform sampler1D rasterTex;

	const float M_PI_2 = 1.57079632679489661923;
	uniform float fov = 1.;
	uniform float ratio = 1.;

	void toCart(in vec2 pVec, out vec3 cartVec)
	{
		cartVec[0] = cos(pVec[0])*cos(pVec[1]);
		cartVec[1] = sin(pVec[0])*cos(pVec[1]);
		cartVec[2] = sin(pVec[1]);
	}

	void toPolar(in vec3 cartVec, out vec2 pVec)
	{
		pVec[0] = atan(cartVec[1], cartVec[0]);
		pVec[1] = asin(cartVec[2]);
	}

	void interpolatedValue(inout float x)
	{
	    x = (M_PI_2 - x) / M_PI_2 / 2.;
			x = texture(rasterTex, x).x;
	}

	void main(void)
	{
		vec2 displayP;
		vec3 carthesic;
		displayP[0] = atan(-v_screenPosition.y, v_screenPosition.x * ratio);
		displayP[1] = M_PI_2 - atan(sqrt(v_screenPosition.x * v_screenPosition.x * ratio * ratio +
					v_screenPosition.y * v_screenPosition.y) * fov);

		toCart(displayP, carthesic);
		carthesic = first * carthesic;
		toPolar(carthesic, displayP);

		float sinResult = sin(displayP[1]);
		displayP[1] = asin((sinResult - beta) / (1 - beta * sinResult));

		toCart(displayP, carthesic);
		carthesic = second * carthesic;
		toPolar(carthesic, displayP);

		interpolatedValue(displayP[1]);
		bool isOnSurface = (displayP[1] > -10.);

		toCart(displayP, carthesic);
		carthesic = third * carthesic;
		toPolar(carthesic, displayP);

		displayP[0] = displayP[0] / M_PI_2 / 4.;
		displayP[1] = displayP[1] / M_PI_2 / 2. + 0.5;
		out_Color = isOnSurface ? texture(tex, displayP) : vec4(0., 0., 0., 1.);
	}
