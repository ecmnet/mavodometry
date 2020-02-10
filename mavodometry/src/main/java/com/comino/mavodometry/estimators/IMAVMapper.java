package com.comino.mavodometry.estimators;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F32;
import georegression.struct.point.Vector4D_F64;

public interface IMAVMapper {

	public boolean update(Vector3D_F32 point);

	public boolean update(Point3D_F64 point);

	public boolean update(Point3D_F64 point, Vector4D_F64 pos);

	public boolean update(float lpos_x, float lpos_y, Point3D_F64 point);

}
