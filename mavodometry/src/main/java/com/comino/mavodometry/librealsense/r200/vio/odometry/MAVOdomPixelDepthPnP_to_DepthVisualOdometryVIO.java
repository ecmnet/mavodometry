/*
 * Copyright (c) 2011-2016, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.comino.mavodometry.librealsense.r200.vio.odometry;


import static boofcv.factory.distort.LensDistortionFactory.narrow;

import java.util.ArrayList;
import java.util.List;

import boofcv.abst.feature.tracker.PointTrack;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.alg.distort.PointToPixelTransform_F32;
import boofcv.alg.geo.DistanceFromModelMultiView;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.sfm.d3.VisOdomPixelDepthPnP;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.distort.Point2Transform2_F32;
import boofcv.struct.distort.Point2Transform2_F64;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageType;
import boofcv.struct.sfm.Point2D3DTrack;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

/**
 * Wrapper around {@link VisOdomPixelDepthPnP} for {@link DepthVisualOdometry}.
 *
 * @author Peter Abeles
 */
//
// this is probably true of other SFM algorithms
public class MAVOdomPixelDepthPnP_to_DepthVisualOdometryVIO<Vis extends ImageBase<Vis>, Depth extends ImageGray<Depth>>
	implements MAVDepthVisualOdometry<Vis,Depth> , AccessPointTracks3D
{
	// low level algorithm
	DepthSparse3D<Depth> sparse3D;
	MAVOdomPixelDepthPnPVIO<Vis> alg;
	DistanceFromModelMultiView<Se3_F64,Point2D3D> distance;
	ImageType<Vis> visualType;
	Class<Depth> depthType;
	boolean success;

	Point2Transform2_F64 leftPixelToNorm = null;
	Point2Transform2_F64 leftNormToPixel = null;

	List<PointTrack> active = new ArrayList<PointTrack>();

	public MAVOdomPixelDepthPnP_to_DepthVisualOdometryVIO(DepthSparse3D<Depth> sparse3D, MAVOdomPixelDepthPnPVIO<Vis> alg,
			                                DistanceFromModelMultiView<Se3_F64, Point2D3D> distance,
													   ImageType<Vis> visualType, Class<Depth> depthType) {
		this.sparse3D = sparse3D;
		this.alg = alg;
		this.distance = distance;
		this.visualType = visualType;
		this.depthType = depthType;
	}

	@Override
	public Point3D_F64 getTrackLocation(int index) {
		try {
			PointTrack t = alg.getTracker().getActiveTracks(null).get(index);
			return ((Point2D3D)t.getCookie()).getLocation();
		} catch( IndexOutOfBoundsException e ) {
			return new Point3D_F64();
		}
	}

	@Override
	public long getTrackId(int index) {
		return active.get(index).featureId;
	}

	@Override
	public List<Point2D_F64> getAllTracks() {
		return (List)active;
	}

	@Override
	public boolean isInlier(int index) {
		Point2D3DTrack t = active.get(index).getCookie();
		return t.lastInlier == alg.getTick();
	}

	@Override
	public boolean isNew(int index) {
		PointTrack t = alg.getTracker().getActiveTracks(null).get(index);
		return alg.getTracker().getNewTracks(null).contains(t);
	}

	@Override
	public void setCalibration(CameraPinholeBrown paramVisual, Point2Transform2_F32 visToDepth) {
		PointToPixelTransform_F32 visToDepth_pixel = new PointToPixelTransform_F32(visToDepth);
		sparse3D.configure(narrow(paramVisual),visToDepth_pixel);

		Point2Transform2_F64 leftPixelToNorm = narrow(paramVisual).undistort_F64(true,false);
		Point2Transform2_F64 leftNormToPixel = narrow(paramVisual).distort_F64(false,true);

		alg.setPixelToNorm(leftPixelToNorm);
		alg.setNormToPixel(leftNormToPixel);

		distance.setIntrinsic(0,paramVisual);
	}

	@Override
	public boolean process(Vis visual, Depth depth, Se3_F64 state) {
		sparse3D.setDepthImage(depth);
		success = alg.process(visual,state);

		active.clear();
		alg.getTracker().getActiveTracks(active);

		return success;
	}

	@Override
	public boolean process(Vis visual, Depth depth) {
		return process(visual,depth,null);
	}

	@Override
	public void reset() {
		alg.reset();
	}

	@Override
	public boolean isFault() {
		return !success;
	}

	@Override
	public Se3_F64 getCameraToWorld() {
		return alg.getCurrToWorld();
	}

	@Override
	public ImageType<Vis> getVisualType() {
		return visualType;
	}

	@Override
	public Class<Depth> getDepthType() {
		return depthType;
	}

	@Override
	public int getInlierCount() {
		return alg.getInlierTracks().size();
	}

	@Override
	public double getQuality() {
		return alg.getQuality();
	}

	@Override
	public void reset(Se3_F64 initialState) {
		alg.reset(initialState);

	}


	@Override
	public Point3D_F64 getPoint3DFromPixel(int pixelx, int pixely) {

		if(sparse3D.process(pixelx, pixely))
			return sparse3D.getWorldPt();

//		if(sparse3D.process(pixelx+dpx, pixely+dpx))
//			return sparse3D.getWorldPt();
//
//		if(sparse3D.process(pixelx-dpx, pixely-dpx))
//			return sparse3D.getWorldPt();
//
//		if(sparse3D.process(pixelx+dpx, pixely-dpx))
//			return sparse3D.getWorldPt();
//
//		if(sparse3D.process(pixelx-dpx, pixely+dpx))
//			return sparse3D.getWorldPt();


	return null;
	}


}
