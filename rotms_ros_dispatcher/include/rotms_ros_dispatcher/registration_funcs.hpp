#pragma once

#include <vector>
#include <tuple>

std::vector<std::vector<double>> getRegistrationResult(std::vector<double> aa, std::vector<double> bb);

std::vector<std::vector<double>> getZeros3by3();

std::vector<std::vector<double>> getZeros(int n);

std::vector<std::vector<double>> getEye3by3();

std::vector<std::vector<double>> getDiag3by3(std::vector<double> v);

std::vector<std::vector<double>> matrixMult3by3(std::vector<std::vector<double>> X, std::vector<std::vector<double>> Y);

std::vector<std::vector<double>> transpose3by3(std::vector<std::vector<double>> A);

double det3by3(std::vector<std::vector<double>> R);

std::vector<double> getCentroid(std::vector<std::vector<double>> A);

std::vector<std::vector<double>> getDeviations(std::vector<std::vector<double>> A, std::vector<double> aCentroid);

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> qrSim3by3(std::vector<std::vector<double>> A);

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> svdSim3by3(std::vector<std::vector<double>> A);