#include <iostream>
#include <string>

using namespace std;

template <typename T>
void c_print(T t);

template <typename T>
void c_print(T t, const string color);

template <typename T, typename R>
void c_print(T t, R r, const string color);

template <typename T, typename R, typename P>
void c_print(T t, R r, P p, const string color);

template <typename T, typename R, typename P, typename F>
void c_print(T t, R r, P p, F f, const string color);

// template <typename T>
// void a_print(T t);

// template <typename T>
// void a_print(T t, const string color);

#include "impl/color_cout.i.hpp"
