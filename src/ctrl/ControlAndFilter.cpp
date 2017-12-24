#include "ctrl/ControlAndFilter.hpp"


namespace sat {
namespace ctrl	{

using utils::Vector3f;
using utils::Vector4f;


MahoneyFilter::MahoneyFilter(float kp, float ki) 
   : twoKp(2 * kp), twoKi(2 * ki), q{ 1.f,0.f,0.f,0.f }, integralFB{0.f,0.f,0.f} {
	
   lastUpdate = std::chrono::high_resolution_clock::now();
	now = std::chrono::high_resolution_clock::now();
}


Vector4f MahoneyFilter::operator()(utils::Matrix<float, 3, 3> vec) {

	now = std::chrono::high_resolution_clock::now();
	const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - lastUpdate).count();
	const auto sampleFreq = float(1.0 / (dt / 10E6));
	lastUpdate = now;

	auto& g = vec[0];
	auto& a = vec[1];
	auto& m = vec[2];

	Vector3f halfe{ 0.f, 0.f, 0.f };

	// Auxiliary variables to avoid repeated arithmetic
	const auto q0q0 = q[0] * q[0];
	const auto q0q1 = q[0] * q[1];
	const auto q0q2 = q[0] * q[2];
	const auto q0q3 = q[0] * q[3];
	const auto q1q1 = q[1] * q[1];
	const auto q1q2 = q[1] * q[2];
	const auto q1q3 = q[1] * q[3];
	const auto q2q2 = q[2] * q[2];
	const auto q2q3 = q[2] * q[3];
	const auto q3q3 = q[3] * q[3];

	// Use magnetometer measurement only when valid (not NaN, 0 or INF)
	if (utils::isnormal(m)) {
		m.normalize();

		// Reference direction of Earth's magnetic field
		const auto hx = 2.0f * (m[0] * (0.5f - q2q2 - q3q3) + m[1] * (q1q2 - q0q3) + m[2] * (q1q3 + q0q2));
		const auto hy = 2.0f * (m[0] * (q1q2 + q0q3) + m[1] * (0.5f - q1q1 - q3q3) + m[2] * (q2q3 - q0q1));
		const auto bx = static_cast<float>(sqrt(hx * hx + hy * hy));
		const auto bz = 2.0f * (m[0] * (q1q3 - q0q2) + m[1] * (q2q3 + q0q1) + m[2] * (0.5f - q1q1 - q2q2));

		// Estimated direction of magnetic field
		const Vector3f halfw{ bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2),
							  bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3),
							  bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2) };

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfe[0] = m[1] * halfw[2] - m[2] * halfw[1];
		halfe[1] = m[2] * halfw[0] - m[0] * halfw[2];
		halfe[2] = m[0] * halfw[1] - m[1] * halfw[0];
	}

	// Compute feedback only if accelerometer measurement valid (not NaN, 0 or INF)
	if (utils::isnormal(a)) {
		a.normalize();

		// Estimated direction of gravity
		const Vector3f halfv{ q1q3 - q0q2,
							       q0q1 + q2q3,
							       q0q0 - 0.5f + q3q3 };

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfe[0] += a[1] * halfv[2] - a[2] * halfv[1];
		halfe[1] += a[2] * halfv[0] - a[0] * halfv[2];
		halfe[2] += a[0] * halfv[1] - a[1] * halfv[0];
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if (halfe != Vector3f{ 0.f, 0.f, 0.f }) {
		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f) {
			integralFB += halfe * ((1.0f / sampleFreq) * twoKi); // integral error scaled by Ki
			g += integralFB; // apply integral feedback
		}
		else {
			integralFB = { 0.f, 0.f, 0.f };
		}

		// Apply proportional feedback
		g += twoKp * halfe;
	}

	// Integrate rate of change of quaternion
	g = g * (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	const auto qa = q[0];
	const auto qb = q[1];
	const auto qc = q[2];
	q[0] += -qb * g[0] - qc * g[1] - q[3] * g[2];
	q[1] +=  qa * g[0] + qc * g[2] - q[3] * g[1];
	q[2] +=  qa * g[1] - qb * g[2] + q[3] * g[0];
	q[3] +=  qa * g[2] + qb * g[1] - qc   * g[0];

	q.normalize();

	return q;
}

void MahoneyFilter::reset() {
	q = { 1.f, 0.f, 0.f, 0.f };
	integralFB.fill(0.0f);
   resetClock();
}

void MahoneyFilter::resetClock() {
   lastUpdate = std::chrono::high_resolution_clock::now();
   now = std::chrono::high_resolution_clock::now();
}

std::unique_ptr<MahoneyFilter> MahoneyFilter::create(float kp, float ki) {
	return std::make_unique<MahoneyFilter>(kp, ki);
}




MadgwickFilter::MadgwickFilter(float beta) : samplePeriod(0.001f), beta(beta), q{ 1.f,0.f,0.f,0.f } {
   now = std::chrono::high_resolution_clock::now();
   lastUpdate = std::chrono::high_resolution_clock::now();
}

utils::Vector4f MadgwickFilter::operator()(utils::Matrix<float, 3, 3> vec) {

   now = std::chrono::high_resolution_clock::now();
   const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - lastUpdate).count();
   samplePeriod = float(dt / 10E6);
   lastUpdate = now;

   if (vec[3] == Vector3f{ 0.f,0.f,0.f })
      Update(vec[0][0], vec[0][1], vec[0][2], 
             vec[1][0], vec[1][1], vec[1][2]);
   else
      Update(vec[0][0], vec[0][1], vec[0][2], 
             vec[1][0], vec[1][1], vec[1][2],
             vec[2][0], vec[2][1], vec[2][2]);

	return q;
}

void MadgwickFilter::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	auto q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability

   const auto _2q1 = 2.f * q1;
   const auto _2q2 = 2.f * q2;
   const auto _2q3 = 2.f * q3;
   const auto _2q4 = 2.f * q4;
   const auto _2q1q3 = 2.f * q1 * q3;
   const auto _2q3q4 = 2.f * q3 * q4;
   const auto q1q1 = q1 * q1;
   const auto q1q2 = q1 * q2;
   const auto q1q3 = q1 * q3;
   const auto q1q4 = q1 * q4;
   const auto q2q2 = q2 * q2;
   const auto q2q3 = q2 * q3;
   const auto q2q4 = q2 * q4;
   const auto q3q3 = q3 * q3;
   const auto q3q4 = q3 * q4;
   const auto q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	auto norm = float(sqrt(ax * ax + ay * ay + az * az));
	if (norm == 0.f) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = float(sqrt(mx * mx + my * my + mz * mz));
	if (norm == 0.f) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	const auto _2q1mx = 2.f * q1 * mx;
	const auto _2q1my = 2.f * q1 * my;
	const auto _2q1mz = 2.f * q1 * mz;
	const auto _2q2mx = 2.f * q2 * mx;
	const auto hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	const auto hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	const auto _2bx = float(sqrt(hx * hx + hy * hy));
	const auto _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	const auto _4bx = 2.f * _2bx;
	const auto _4bz = 2.f * _2bz;

	// Gradient decent algorithm corrective step
   auto s1 = -_2q3 * (2.f * q2q4 - _2q1q3 - ax) + _2q2 * (2.f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
   auto s2 = _2q4 * (2.f * q2q4 - _2q1q3 - ax) + _2q1 * (2.f * q1q2 + _2q3q4 - ay) - 4.f * q2 * (1 - 2.f * q2q2 - 2.f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
   auto s3 = -_2q1 * (2.f * q2q4 - _2q1q3 - ax) + _2q4 * (2.f * q1q2 + _2q3q4 - ay) - 4.f * q3 * (1 - 2.f * q2q2 - 2.f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
   auto s4 = _2q2 * (2.f * q2q4 - _2q1q3 - ax) + _2q3 * (2.f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = 1.f / float(sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4));    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	const auto qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	const auto qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	const auto qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	const auto qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * samplePeriod;
	q2 += qDot2 * samplePeriod;
	q3 += qDot3 * samplePeriod;
	q4 += qDot4 * samplePeriod;
	norm = 1.f / float(sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4));    // normalise quaternion
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void MadgwickFilter::Update(float gx, float gy, float gz, float ax, float ay, float az)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1 = 2.f * q1;
	float _2q2 = 2.f * q2;
	float _2q3 = 2.f * q3;
	float _2q4 = 2.f * q4;
	float _4q1 = 4.f * q1;
	float _4q2 = 4.f * q2;
	float _4q3 = 4.f * q3;
	float _8q2 = 8.f * q2;
	float _8q3 = 8.f * q3;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = float(sqrt(ax * ax + ay * ay + az * az));
	if (norm == 0.f) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Gradient decent algorithm corrective step
	s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
	s2 = _4q2 * q4q4 - _2q4 * ax + 4.f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
	s3 = 4.f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
	s4 = 4.f * q2q2 * q4 - _2q2 * ax + 4.f * q3q3 * q4 - _2q3 * ay;
	norm = 1.f / float(sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4));    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * samplePeriod;
	q2 += qDot2 * samplePeriod;
	q3 += qDot3 * samplePeriod;
	q4 += qDot4 * samplePeriod;
	norm = 1.f / float(sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4));    // normalise quaternion
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void MadgwickFilter::reset() {
	q = { 1.f, 0.f, 0.f, 0.f };
   resetClock();
}

void MadgwickFilter::resetClock() {
   now = std::chrono::high_resolution_clock::now();
   lastUpdate = std::chrono::high_resolution_clock::now();
}

std::unique_ptr<MadgwickFilter> MadgwickFilter::create() {
	return std::make_unique<MadgwickFilter>();
}

/*
/// <summary>
 /// MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms.
 /// </summary>
 /// <remarks>
 /// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 /// </remarks>
public class MadgwickAHRS
{
	/// <summary>
	/// Gets or sets the sample period.
	/// </summary>
	public float SamplePeriod{ get; set; }

		/// <summary>
		/// Gets or sets the algorithm gain beta.
		/// </summary>
	public float Beta{ get; set; }

		/// <summary>
		/// Gets or sets the Quaternion output.
		/// </summary>
	public float[] Quaternion{ get; set; }

		/// <summary>
		/// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
		/// </summary>
		/// <param name="samplePeriod">
		/// Sample period.
		/// </param>
		public MadgwickAHRS(float samplePeriod)
		: this(samplePeriod, 1f)
	{
	}

	/// <summary>
	/// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
	/// </summary>
	/// <param name="samplePeriod">
	/// Sample period.
	/// </param>
	/// <param name="beta">
	/// Algorithm gain beta.
	/// </param>
	public MadgwickAHRS(float samplePeriod, float beta)
	{
		SamplePeriod = samplePeriod;
		Beta = beta;
		Quaternion = new float[] { 1f, 0f, 0f, 0f };
	}

	/// <summary>
	/// Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
	/// </summary>
	/// <param name="gx">
	/// Gyroscope x axis measurement in radians/s.
	/// </param>
	/// <param name="gy">
	/// Gyroscope y axis measurement in radians/s.
	/// </param>
	/// <param name="gz">
	/// Gyroscope z axis measurement in radians/s.
	/// </param>
	/// <param name="ax">
	/// Accelerometer x axis measurement in any calibrated units.
	/// </param>
	/// <param name="ay">
	/// Accelerometer y axis measurement in any calibrated units.
	/// </param>
	/// <param name="az">
	/// Accelerometer z axis measurement in any calibrated units.
	/// </param>
	/// <param name="mx">
	/// Magnetometer x axis measurement in any calibrated units.
	/// </param>
	/// <param name="my">
	/// Magnetometer y axis measurement in any calibrated units.
	/// </param>
	/// <param name="mz">
	/// Magnetometer z axis measurement in any calibrated units.
	/// </param>
	/// <remarks>
	/// Optimised for minimal arithmetic.
	/// Total ±: 160
	/// Total *: 172
	/// Total /: 5
	/// Total sqrt: 5
	/// </remarks> 
	public void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
	{
		float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
		float norm;
		float hx, hy, _2bx, _2bz;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1mx;
		float _2q1my;
		float _2q1mz;
		float _2q2mx;
		float _4bx;
		float _4bz;
		float _2q1 = 2f * q1;
		float _2q2 = 2f * q2;
		float _2q3 = 2f * q3;
		float _2q4 = 2f * q4;
		float _2q1q3 = 2f * q1 * q3;
		float _2q3q4 = 2f * q3 * q4;
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0f) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Normalise magnetometer measurement
		norm = (float)Math.Sqrt(mx * mx + my * my + mz * mz);
		if (norm == 0f) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		mx *= norm;
		my *= norm;
		mz *= norm;

		// Reference direction of Earth's magnetic field
		_2q1mx = 2f * q1 * mx;
		_2q1my = 2f * q1 * my;
		_2q1mz = 2f * q1 * mz;
		_2q2mx = 2f * q2 * mx;
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = (float)Math.Sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2f * _2bx;
		_4bz = 2f * _2bz;

		// Gradient decent algorithm corrective step
		s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		norm = 1f / (float)Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * SamplePeriod;
		q2 += qDot2 * SamplePeriod;
		q3 += qDot3 * SamplePeriod;
		q4 += qDot4 * SamplePeriod;
		norm = 1f / (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		Quaternion[0] = q1 * norm;
		Quaternion[1] = q2 * norm;
		Quaternion[2] = q3 * norm;
		Quaternion[3] = q4 * norm;
	}

	/// <summary>
	/// Algorithm IMU update method. Requires only gyroscope and accelerometer data.
	/// </summary>
	/// <param name="gx">
	/// Gyroscope x axis measurement in radians/s.
	/// </param>
	/// <param name="gy">
	/// Gyroscope y axis measurement in radians/s.
	/// </param>
	/// <param name="gz">
	/// Gyroscope z axis measurement in radians/s.
	/// </param>
	/// <param name="ax">
	/// Accelerometer x axis measurement in any calibrated units.
	/// </param>
	/// <param name="ay">
	/// Accelerometer y axis measurement in any calibrated units.
	/// </param>
	/// <param name="az">
	/// Accelerometer z axis measurement in any calibrated units.
	/// </param>
	/// <remarks>
	/// Optimised for minimal arithmetic.
	/// Total ±: 45
	/// Total *: 85
	/// Total /: 3
	/// Total sqrt: 3
	/// </remarks>
	public void Update(float gx, float gy, float gz, float ax, float ay, float az)
	{
		float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
		float norm;
		float s1, s2, s3, s4;
		float qDot1, qDot2, qDot3, qDot4;

		// Auxiliary variables to avoid repeated arithmetic
		float _2q1 = 2f * q1;
		float _2q2 = 2f * q2;
		float _2q3 = 2f * q3;
		float _2q4 = 2f * q4;
		float _4q1 = 4f * q1;
		float _4q2 = 4f * q2;
		float _4q3 = 4f * q3;
		float _8q2 = 8f * q2;
		float _8q3 = 8f * q3;
		float q1q1 = q1 * q1;
		float q2q2 = q2 * q2;
		float q3q3 = q3 * q3;
		float q4q4 = q4 * q4;

		// Normalise accelerometer measurement
		norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);
		if (norm == 0f) return; // handle NaN
		norm = 1 / norm;        // use reciprocal for division
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Gradient decent algorithm corrective step
		s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
		s2 = _4q2 * q4q4 - _2q4 * ax + 4f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
		s3 = 4f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
		s4 = 4f * q2q2 * q4 - _2q2 * ax + 4f * q3q3 * q4 - _2q3 * ay;
		norm = 1f / (float)Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
		s4 *= norm;

		// Compute rate of change of quaternion
		qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
		qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
		qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
		qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

		// Integrate to yield quaternion
		q1 += qDot1 * SamplePeriod;
		q2 += qDot2 * SamplePeriod;
		q3 += qDot3 * SamplePeriod;
		q4 += qDot4 * SamplePeriod;
		norm = 1f / (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
		Quaternion[0] = q1 * norm;
		Quaternion[1] = q2 * norm;
		Quaternion[2] = q3 * norm;
		Quaternion[3] = q4 * norm;
	}
}
 */

} // namespace ctrl
} // namespace sat