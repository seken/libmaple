#pragma once

#include <cstdlib>

/**
 * Simplistic PID class
 *
 * Doesn't use delta time, relies on being called at a stable rate...
 **/
template <class T>
class PID {
	public:
		PID(const T &p, const T &i, const T &d, const T &iLimit);

		T calculate(const T &demand, const T &actual);
		void reset();
		T getValue() const;
		
	private:
		const T m_p;
		const T m_i;
		const T m_d;
		const T m_iLimit;
		T m_integral;
		T m_lastError;
		T m_lastResult;
};

template<typename T>
PID<T>::PID(const T &p, const T &i, const T &d, const T &iLimit) :
		m_p(p),
		m_i(i),
		m_d(d),
		m_iLimit(iLimit),
		m_integral(0),
		m_lastError(0),
		m_lastResult(0) {}

template<typename T>
T PID<T>::getValue() const {
	return m_lastResult;
}

template<typename T>
T PID<T>::calculate(const T &demand, const T &actual) {
	const T error = demand - actual;
	const T deltaError = error - m_lastError;
	m_lastError = error;
	m_integral += error;

	if (abs(m_integral) > m_iLimit) {
		m_integral = (m_integral < 0) ? -m_iLimit : m_iLimit;
	}

	m_lastResult = (error * m_p) + (m_integral * m_i) + (deltaError * m_d);
	return m_lastResult;
}

template<typename T>
void PID<T>::reset() {
	m_integral = 0;
	m_lastError = 0;
}
