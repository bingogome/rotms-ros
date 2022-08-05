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

#include "flag_machine.hpp"
#include "flag_machine_digitization.hpp"
#include <vector>

//
FlagMachineDigitization::FlagMachineDigitization() : FlagMachineBase()
{
    flag_all_digitized_ = false;
    temp_dig_idx_ = -1
}

bool FlagMachineDigitization::flag_all_digitized_;
std::vector<bool> FlagMachineDigitization::temp_dig_flag_arr_;
int temp_dig_idx_;

void FlagMachineDigitization::ComfirmAllDigitized(){flag_all_digitized_=true;}
void FlagMachineDigitization::UnconfirmAllDigitized(){flag_all_digitized_=false;}
bool FlagMachineDigitization::GetAllDigitized(){return flag_all_digitized_;}

void FlagMachineDigitization::InitializeDigFlagArr()
{
    std::vector<bool> new_arr;
    temp_dig_flag_arr_ = new_arr;
}

void FlagMachineDigitization::ClearDigFlagArr()
{
    for(int i=0;i<temp_dig_flag_arr_.size();i++)
    {
        temp_dig_flag_arr_[i] = false;
    }
}

void FlagMachineDigitization::ResetDigFlagArrAt(int idx)
{
    temp_dig_flag_arr_[idx] = false;
}

void FlagMachineDigitization::ResetDigFlagArrAt()
{
    FlagMachineDigitization::ResetDigFlagArrAt(temp_dig_idx_);
}

void FlagMachineDigitization::SetDigFlagArrAt(int idx)
{
    temp_dig_flag_arr_[idx] = true;
}

void FlagMachineDigitization::SetDigFlagArrAt()
{
    FlagMachineDigitization::SetDigFlagArrAt(temp_dig_idx_);
}

std::vector<bool> FlagMachineDigitization::GetDigFlagArr()
{
    return temp_dig_flag_arr_;
}

void FlagMachineDigitization::SetTempDigitizationIdx(int idx)
{
    temp_dig_idx_ = idx;
}

void FlagMachineDigitization::ClearTempDigitizationIdx()
{
    temp_dig_idx_ = -1;
}