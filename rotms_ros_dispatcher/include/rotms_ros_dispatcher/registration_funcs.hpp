#pragma once

#include <vector>
#include <tuple>

std::vector<double> rotm2quat(const std::vector<std::vector<double>>& R);

void SaveRegistrationData(
    const std::vector<double>& quat, const std::vector<double>& p, const std::string& f);

std::string FormatDouble2String(double a, int dec);

std::string GetTimeString();

std::tuple<std::vector<std::vector<double>>, std::vector<double>> getRegistrationResult(
    const std::vector<std::vector<double>>& a, const std::vector<std::vector<double>>& b);

std::vector<std::vector<double>> getZeros3by3();

std::vector<std::vector<double>> getZeros(int n);

std::vector<std::vector<double>> getEye3by3();

std::vector<std::vector<double>> getDiag3by3(const std::vector<double>& v);

std::vector<std::vector<double>> matrixMult3by3(const std::vector<std::vector<double>>& X, const std::vector<std::vector<double>>& Y);

std::vector<std::vector<double>> transpose3by3(const std::vector<std::vector<double>>& A);

double det3by3(const std::vector<std::vector<double>>& R);

std::vector<double> getCentroid(const std::vector<std::vector<double>>& A);

std::vector<std::vector<double>> getDeviations(const std::vector<std::vector<double>>& A, const std::vector<double>& aCentroid);

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> qrSim3by3(const std::vector<std::vector<double>>& A);

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> svdSim3by3(const std::vector<std::vector<double>>& A);