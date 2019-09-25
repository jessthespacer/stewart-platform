#include <Geometry.h>
#include <math.h>

// Calculate sum of squared components of vector or square of norm
static float sumsq(Point v) {
	return pow(v.X(), 2) + pow(v.Y(), 2) + pow(v.Z(), 2);
}

// T in mm, Pang in radians
// alph will have angles written to it if horn == true
// alph will have leg lengths written to it if horn == false
void getAlpha(Point T, float Pang[], \
	bool horn, float alph[], float beta[], Point P[], Point B[], float s, float a) {

	// Define platform rotation matrix
	Rotation ProtB;
	ProtB.RotateX(Pang[0]);
	ProtB.RotateY(Pang[1]);
	ProtB.RotateZ(Pang[2]);

	Point Pxyz[6];
	Point l;
	float alphn, betan, en, fn, gn;

	// Iterate over all legs
	for (int n = 0; n < 6; ++n) {
		// Rotate platform vectors in global coords.
		Pxyz[n] = ProtB * P[n];

		// Calculate leg vectors
		l = T + Pxyz[n] - B[n];

		if (horn) {
			betan = beta[n];
			
			// Calculate servo angles
			en = 2 * a * l.Z();
			fn = 2 * a * (cos(betan) * l.X() + sin(betan) * l.Y());
			gn = sumsq(l) - (pow(s, 2) - pow(a, 2));

			alphn = asin(gn / sqrt(pow(en, 2) + pow(fn, 2))) - atan2(fn, en);
			
			// Write to output
			alph[n] = alphn;
		}
		else {
			// Get leg lengths
			alph[n] = sqrt(sumsq(l));
		}
	}
	return;
}
