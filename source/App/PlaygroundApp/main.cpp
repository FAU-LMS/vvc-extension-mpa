//
// Created by regensky on 02.03.21.
//

#include <iostream>
#include <chrono>
#include <random>
#include <bitset>
#include "Coordinate.h"
#include "Projection.h"
#include "MVReprojection.h"
#include "LookupTable.h"
using namespace std;

ArrayXXTCoordPtrPair toPtrPair(Array2TCoord array2TCoord) {
  ArrayXXTCoord coord0(Eigen::Index(1), Eigen::Index(1));
  ArrayXXTCoord coord1(Eigen::Index(1), Eigen::Index(1));
  coord0.coeffRef(0) = array2TCoord.coeff(0);
  coord1.coeffRef(0) = array2TCoord.coeff(1);
  ArrayXXTCoordPtr coord0Ptr = std::make_shared<ArrayXXTCoord>(coord0);
  ArrayXXTCoordPtr coord1Ptr = std::make_shared<ArrayXXTCoord>(coord1);
  return {coord0Ptr, coord1Ptr};
}

ArrayXXTCoordPtr toPtr(TCoord tCoord) {
  ArrayXXTCoord coord(Eigen::Index(1), Eigen::Index(1));
  coord.coeffRef(0) = tCoord;
  ArrayXXTCoordPtr coordPtr = std::make_shared<ArrayXXTCoord>(coord);
  return coordPtr;
}

void printArray(Array2TCoord array) {
  std::cout << "(" << array.x() << ", " << array.y() << ")\n";
}

void printArray(Array3TCoord array) {
  std::cout << "(" << array.x() << ", " << array.y() << ", " << array.z() << ")\n";
}

void printArrayPtr(ArrayXXTCoordPtrPair array) {
  std::cout << "(" << std::get<0>(array)->coeff(0) << ", " << std::get<1>(array)->coeff(0) << ")\n";
}

void printArrayPtr(ArrayXXTCoordPtrTriple array) {
  std::cout << "(" << std::get<0>(array)->coeff(0) << ", " << std::get<1>(array)->coeff(0) << ", " << std::get<2>(array)->coeff(0) << ")\n";
}

bool compare(Array2TCoord array, ArrayXXTCoordPtrPair arrayPtr) {
  TCoord diffX = array.x() - std::get<0>(arrayPtr)->coeff(0);
  TCoord diffY = array.y() - std::get<1>(arrayPtr)->coeff(0);

  return (abs(diffX) < 1 && abs(diffY) < 1);
}

bool compare(Array3TCoord array, ArrayXXTCoordPtrTriple arrayPtr) {
  TCoord diffX = array.x() - std::get<0>(arrayPtr)->coeff(0);
  TCoord diffY = array.y() - std::get<1>(arrayPtr)->coeff(0);
  TCoord diffZ = array.z() - std::get<2>(arrayPtr)->coeff(0);

  return (abs(diffX) < 1 && abs(diffY) < 1 && abs(diffZ) < 1);
}

std::random_device randomDevice;
std::mt19937 generator(randomDevice());
std::uniform_real_distribution<TCoord> distribution(-300, 300);

Array2TCoord randCoord() {
  return {distribution(generator), distribution(generator)};
}

void checkMatch(Array2TCoord array, ArrayXXTCoordPtrPair arrayPtr) {
  if (!compare(array, arrayPtr))
  {
    //printArray2TCoord(cart2DOrig);
    printArray(array);
    printArrayPtr(arrayPtr);
    std::cout << "\n";
  }
}

void checkMatch(Array3TCoord array, ArrayXXTCoordPtrTriple arrayPtr) {
  if (!compare(array, arrayPtr))
  {
    //printArray2TCoord(cart2DOrig);
    printArray(array);
    printArrayPtr(arrayPtr);
    std::cout << "\n";
  }
}

int main(int argc, char* argv[]) {
  std::cout << (1088./5.2)*1.8 << "\n";
  std::cout << FloatingFixedConversion::floatingToFixed((1088./5.2)*1.8, 16) << "\n";
  std::cout << FloatingFixedConversion::floatingToFixed(543.5, 16) << "\n";
  std::cout << FloatingFixedConversion::floatingToFixed(543.5, 16) << "\n";

  TCoord a = -4.25;
  int fixed = FloatingFixedConversion::floatingToFixed(a, 16);
  TCoord aRes = FloatingFixedConversion::fixedToFloating(fixed, 16);

  ArrayXXTCoordPtr aPtr = toPtr(a);
  ArrayXXFixedPtr fixedPtr = FloatingFixedConversion::floatingToFixed(aPtr, 16);
  aPtr = FloatingFixedConversion::fixedToFloating(fixedPtr, 16);
  std::cout << aRes << ", " << *aPtr << "\n";
  std::cout << fixed << ", " << *fixedPtr << "\n\n";

  LookupTable lut([](TCoord input){return input*input*input;}, {1, 8}, 100);
  ArrayXXTCoordPtr inputs = std::make_shared<ArrayXXTCoord>(ArrayXTCoord::LinSpaced(22, 1, 8));
  std::cout << *inputs << "\n";
  std::cout << *lut.lookup(inputs) << "\n";
  std::cout << *lut.inverseLookup(lut.lookup(inputs)) << "\n";

  TCoord pxPerMm = 1088./5.2;
  TCoord focalLength = 1.8;
  ArrayXTCoord coefficients(Eigen::Index(1), Eigen::Index(11));
  coefficients << 1.6278e-6, 1.5678, 0.0043719, -0.10717, 0.43818, -0.78004, 1.136, -1.2132, 0.75722, -0.24354, 0.03145;
  coefficients = coefficients * pxPerMm;
  Array2TCoord center(542.394470, 575.214839);

  for(int i=0; i<coefficients.size(); ++i) {
    std::cout << FloatingFixedConversion::floatingToFixed(coefficients(i), 16) << ",";
  }
  std::cout << "\n" << FloatingFixedConversion::floatingToFixed(center(0), 16) << ", " << FloatingFixedConversion::floatingToFixed(center(1), 16) << "\n";

  CalibratedProjection calibrated(pxPerMm*focalLength, center, coefficients);
  std::cout << calibrated.fromSphere(calibrated.toSphere(Array2TCoord(513.31, 823.23))) << "\n";

  EquisolidProjection equisolid((1088./5.2)*1.8, Array2TCoord(0, 0));
  PerspectiveProjection perspective((1088./5.2)*1.8, Array2TCoord(0, 0));

  for (int i=0; i<100000; ++i) {
    Array2TCoord cart2DOrig = randCoord();
    ArrayXXTCoordPtrPair cart2DArrayOrig = toPtrPair(cart2DOrig);

    Array3TCoord spherical = equisolid.toSphere(cart2DOrig);
    ArrayXXTCoordPtrTriple sphericalArray = equisolid.toSphere(cart2DArrayOrig);

    std::pair<Array2TCoord , bool> cart2DVip = perspective.fromSphere(spherical);
    Array2TCoord cart2D = std::get<0>(cart2DVip);
    bool vip = std::get<1>(cart2DVip);
    std::pair<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> cart2DVipArray = perspective.fromSphere(sphericalArray);
    ArrayXXTCoordPtrPair cart2DArray = std::get<0>(cart2DVipArray);
    ArrayXXBoolPtr vipArray = std::get<1>(cart2DVipArray);

    spherical = perspective.toSphere(cart2D, vip);
    sphericalArray = perspective.toSphere(cart2DArray, vipArray);

    cart2D = equisolid.fromSphere(spherical);
    cart2DArray = equisolid.fromSphere(sphericalArray);

    checkMatch(cart2D, cart2DArray);
  }
}
