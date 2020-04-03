package com.comino.mavodometry.estimators;

import georegression.struct.point.Point3D_F64;

public interface ITargetListener {

	public boolean update(Point3D_F64 point, Point3D_F64 body);

}
