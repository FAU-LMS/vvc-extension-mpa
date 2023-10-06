//
// Created by regensky on 8.10.20.
//

#pragma once

#include "CommonDef.h"
#include "Coordinate.h"
#include "LookupTable.h"


/// Projection baseclass
class Projection {
public:

  explicit Projection(TCoord focalLength) : m_focalLength(focalLength) {}
  virtual ~Projection() = default;

  virtual ArrayXXTCoordPtrTriple toSphere(ArrayXXTCoordPtrPair cart2D) const = 0;
  virtual Array3TCoord toSphere(const Array2TCoord &cart2D) const = 0;

  virtual ArrayXXTCoordPtrPair fromSphere(ArrayXXTCoordPtrTriple cart3D) const = 0;
  virtual Array2TCoord fromSphere(const Array3TCoord &cart3D) const = 0;

  TCoord focalLength() const { return m_focalLength; }

protected:
  TCoord m_focalLength;
};


/// Radial projection baseclass
class RadialProjection : public Projection {
public:

  RadialProjection(TCoord focalLength, const Array2TCoord &opticalCenter) : Projection(focalLength), m_opticalCenter(opticalCenter) {}

  ArrayXXTCoordPtrTriple toSphere(ArrayXXTCoordPtrPair cart2D) const override;
  Array3TCoord toSphere(const Array2TCoord &cart2D) const override;

  ArrayXXTCoordPtrPair fromSphere(ArrayXXTCoordPtrTriple cart3D) const override;
  Array2TCoord fromSphere(const Array3TCoord &cart3D) const override;

  virtual ArrayXXTCoordPtr radius(ArrayXXTCoordPtr theta) const = 0;
  virtual TCoord radius(TCoord theta) const = 0;

  virtual ArrayXXTCoordPtr theta(ArrayXXTCoordPtr radius) const = 0;
  virtual TCoord theta(TCoord radius) const = 0;

protected:
  Array2TCoord m_opticalCenter;
};


/// Equisolid projection
class EquisolidProjection : public RadialProjection {
public:
  EquisolidProjection(TCoord focalLength, const Array2TCoord &opticalCenter) : RadialProjection(focalLength, opticalCenter) {}

  ArrayXXTCoordPtr radius(ArrayXXTCoordPtr theta) const override;
  TCoord radius(TCoord theta) const override;

  ArrayXXTCoordPtr theta(ArrayXXTCoordPtr radius) const override;
  TCoord theta(TCoord radius) const override;
};


/// Calibrated projection
class CalibratedProjection : public RadialProjection {
public:
  CalibratedProjection(TCoord focalLength, const Array2TCoord &opticalCenter, const ArrayXTCoord &coefficients) : RadialProjection(focalLength, opticalCenter), m_coefficients(coefficients) {
      init();
  }

  ArrayXXTCoordPtr radius(ArrayXXTCoordPtr theta) const override;
  TCoord radius(TCoord theta) const override;

  ArrayXXTCoordPtr theta(ArrayXXTCoordPtr radius) const override;
  TCoord theta(TCoord radius) const override;

protected:
  void init();
  TCoord polynomial(TCoord value);

protected:
  ArrayXTCoord m_coefficients;
  LookupTable m_lut;
};


/// Perspective projection
class PerspectiveProjection {
public:
  PerspectiveProjection(): m_focalLength(0), m_opticalCenter(Array2TCoord(0, 0)) {}
  PerspectiveProjection(TCoord focalLength, const Array2TCoord &opticalCenter) : m_focalLength(focalLength), m_opticalCenter(opticalCenter) {}

  ArrayXXTCoordPtrTriple toSphere(ArrayXXTCoordPtrPair cart2D, ArrayXXBoolPtr virtualImagePlane) const;
  Array3TCoord toSphere(Array2TCoord cart2D, bool virtualImagePlane) const;

  std::pair<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> fromSphere(ArrayXXTCoordPtrTriple cart3D) const;
  std::pair<Array2TCoord, bool> fromSphere(Array3TCoord cart3D) const;

  ArrayXXTCoordPtr radius(ArrayXXTCoordPtr theta) const;
  TCoord radius(TCoord theta) const;

  ArrayXXTCoordPtr theta(ArrayXXTCoordPtr radius) const;
  TCoord theta(TCoord radius) const;

protected:
  TCoord m_focalLength;
  Array2TCoord m_opticalCenter;
};

/// Equirectangular projection
class EquirectangularProjection : public Projection {
public:

  EquirectangularProjection(const Size &resolution) : Projection(TCoord(1. / std::tan(M_PI/resolution.height))), m_resolution(resolution) {}

  ArrayXXTCoordPtrTriple toSphere(ArrayXXTCoordPtrPair cart2D) const override;
  Array3TCoord toSphere(const Array2TCoord &cart2D) const override;

  ArrayXXTCoordPtrPair fromSphere(ArrayXXTCoordPtrTriple cart3D) const override;
  Array2TCoord fromSphere(const Array3TCoord &cart3D) const override;

protected:
  Size m_resolution;
};
