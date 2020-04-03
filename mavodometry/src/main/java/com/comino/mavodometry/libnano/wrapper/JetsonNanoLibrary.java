package com.comino.mavodometry.libnano.wrapper;
import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.Pointer;
import com.sun.jna.PointerType;
import com.sun.jna.Structure;
import com.sun.jna.ptr.PointerByReference;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;

public interface JetsonNanoLibrary extends Library {


	public static final int		NETWORK_TYPE_CUSTOM 			= 0;
	public static final int		NETWORK_TYPE_COCO_AIRPLANE 	  	= 1;
	public static final int		NETWORK_TYPE_COCO_BOTTLE 	  	= 2;
	public static final int		NETWORK_TYPE_COCO_CHAIR 	  	= 3;
	public static final int		NETWORK_TYPE_COCO_DOG	  		= 4;
	public static final int		NETWORK_TYPE_FACENET	  		= 5;
	public static final int		NETWORK_TYPE_PEDNET		  		= 6;
	public static final int		NETWORK_TYPE_PEDNET_MULTI	  	= 7;

	public static final int		NETWORK_TYPE_MOBILENET_V1 		= 8;
	public static final int		NETWORK_TYPE_MOBILENET_V2 		= 9;
	public static final int		NETWORK_TYPE_INCEPTION_V2 		= 10;



	public static final String JNA_LIBRARY_NAME = "jetsonNano";
	public static final NativeLibrary JNA_NATIVE_LIB = NativeLibrary.getInstance(JetsonNanoLibrary.JNA_LIBRARY_NAME);
	public static final JetsonNanoLibrary INSTANCE = (JetsonNanoLibrary)Native.loadLibrary(JetsonNanoLibrary.JNA_LIBRARY_NAME, JetsonNanoLibrary.class);
	public static class Result extends Structure {
		public int Instance;
		public int ClassID;
		public float Confidence;
		public float Left;
		public float Right;
		public float Top;
		public float Bottom;
		public Result() {
			super();
		}
		protected List<? > getFieldOrder() {
			return Arrays.asList("Instance", "ClassID", "Confidence", "Left", "Right", "Top", "Bottom");
		}
		public Result(int Instance, int ClassID, float Confidence, float Left, float Right, float Top, float Bottom) {
			super();
			this.Instance = Instance;
			this.ClassID = ClassID;
			this.Confidence = Confidence;
			this.Left = Left;
			this.Right = Right;
			this.Top = Top;
			this.Bottom = Bottom;
		}
		public Result(Pointer peer) {
			super(peer);
		}
		public static class ByReference extends Result implements Structure.ByReference {

		};
		public static class ByValue extends Result implements Structure.ByValue {

		};
	};

	PointerByReference instance(int model_type, float threshold, int width, int height);

	PointerByReference instanceCustom(Pointer prototxt_path, Pointer model_path, Pointer class_labels, Pointer out_layer_name,
			                          float threshold, int width, int height);


	Pointer getClassDescription(PointerByReference det, int ClassID);


	int detect(PointerByReference det, ByteBuffer img, JetsonNanoLibrary.Result result, int overlay);


	int detect(PointerByReference det, Pointer img, JetsonNanoLibrary.Result result, int overlay);
	public static class detectNet extends PointerType {
		public detectNet(Pointer address) {
			super(address);
		}
		public detectNet() {
			super();
		}
	};
}
