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
#include "flag_machine.hpp"
#include <vector>

class FlagMachineDigitization : public FlagMachineBase
{

public:

    FlagMachineDigitization();

    static void ComfirmAllDigitized();
    static void UnconfirmAllDigitized();
    static bool GetAllDigitized();

    static void InitializeDigFlagArr();
    static void ClearDigFlagArr();
    static void ResetDigFlagArrAt(int idx);
    static void SetDigFlagArrAt(int idx);
    static void ResetDigFlagArrAt();
    static void SetDigFlagArrAt();
    static std::vector<bool> GetDigFlagArr();

    static void SetTempDigitizationIdx(int idx);
    static void ClearTempDigitizationIdx();

private:

    static bool flag_all_digitized_;
    static std::vector<bool> temp_dig_flag_arr_;

    static int temp_dig_idx_;
};