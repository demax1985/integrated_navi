// Copyright 2021 demax
#ifndef INCLUDE_INTEGRATED_NAVIGATION_H_
#define INCLUDE_INTEGRATED_NAVIGATION_H_

#include <memory>

#include "sins/sins.h"
using sins::SINS;
class IntegratedNavigation {
 private:
  std::shared_ptr<SINS> pSINS_;

 public:
  IntegratedNavigation(/* args */);
  ~IntegratedNavigation();
};

IntegratedNavigation::IntegratedNavigation(/* args */) {}

IntegratedNavigation::~IntegratedNavigation() {}

#endif  // INCLUDE_INTEGRATED_NAVIGATION_H_
