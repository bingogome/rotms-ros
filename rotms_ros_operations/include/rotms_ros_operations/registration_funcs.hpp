/***
MIT License

Copyright (c) 2022 Yihao Liu, Johns Hopkins University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
***/

#pragma once

#include <vector>
#include <tuple>

typedef std::vector<std::vector<double>> Matrixnbym;

std::vector<double> rotm2quat(const Matrixnbym& R);

void SaveRegistrationData(
    const std::vector<double>& quat, const std::vector<double>& p, const std::string& f);

std::string FormatDouble2String(double a, int dec);

std::string GetTimeString();

std::tuple<Matrixnbym, std::vector<double>> getRegistrationResult(
    const Matrixnbym& a, const Matrixnbym& b);

Matrixnbym getZeros3by3();

Matrixnbym getZeros(int n);

Matrixnbym getEye3by3();

Matrixnbym getEye(int n);

Matrixnbym getDiag3by3(const std::vector<double>& v);

Matrixnbym matrixMult3by3(const Matrixnbym& X, const Matrixnbym& Y);

Matrixnbym matrixMult(const Matrixnbym& X, const Matrixnbym& Y);

Matrixnbym transpose3by3(const Matrixnbym& A);

double det3by3(const Matrixnbym& R);

std::vector<double> getCentroid(const Matrixnbym& A);

Matrixnbym getDeviations(const Matrixnbym& A, const std::vector<double>& aCentroid);

std::tuple<Matrixnbym, Matrixnbym> qrSim3by3(const Matrixnbym& A);

Matrixnbym qrhelper (const std::vector<double>& a);

std::tuple<Matrixnbym, Matrixnbym, Matrixnbym> svdSim3by3(const Matrixnbym& A);