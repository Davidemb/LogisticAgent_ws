#pragma once
#include <...>

namespace coo
{
    class COO
    {
        public:
            COO(ros::NodeHandle &nh_);
            ~COO();

            void task_TP_Callback(const typemessagePtr &msg);

        private:
            ros::Subscriber sub_task_TP;
            ros::Publisher  pub_mission_Agent;
    }
}// namespace coo

#include "impl/AgentCoo.i.hpp"
/* 
#include <iostream>

void printSet(int array[],int size){
    int i;

    for (i=1;i<=size;i++)
        std::cout << array[i] << " ";
    std::cout << std::endl;

    return;
}

void printPowerset (int n){
    int stack[10],k;

    stack[0]=0; /* 0 is not considered as part of the set 
    k = 0;

    while(1){
        if (stack[k]<n){
            stack[k+1] = stack[k] + 1;
            k++;
        }

        else{
            stack[k-1]++;
            k--;
        }

        if (k==0)
            break;

        printSet(stack,k);
    }

    return;
}

void powersetRec(int s[], int k, int m, int n) 
{
    if (m <= n) {
        s[k+1] = m ;
        printSet(s, k+1) ; 
        powersetRec(s, k+1, m+1, n) ; // with m 
        powersetRec(s, k, m+1, n) ;   //without m 
    }
}

int main(){
    int stack[10];
    // printPowerset(5);
    powersetRec(stack,0,1,2);

    return 0;
} */