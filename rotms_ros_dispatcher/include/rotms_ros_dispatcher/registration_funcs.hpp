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