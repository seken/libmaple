#pragma once

template<class T>
T MAX(const T &a, const T &b, const T &c, const T &d) {
	if (a>b && a>c && a>d) {
		return a;
	} else if (b>c && b>d) {
		return b;
	} else if (c > d) {
		return c;
	}
	return d;
}

template<class T>
T MIN(const T &a, const T &b, const T &c, const T &d) {
	if (a<b && a<c && a<d) {
		return a;
	} else if (b<c && b<d) {
		return b;
	} else if (c < d) {
		return c;
	}
	return d;
}
