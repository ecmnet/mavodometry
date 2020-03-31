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
	/**
	 * Original signature : <code>detectNet* instance(int, char**)</code><br>
	 * <i>native declaration : line 59</i>
	 */
	PointerByReference instance(int argc, PointerByReference argv, int width, int height);

	/**
	 * Original signature : <code>char* getClassDescription(detectNet*, uint32_t)</code><br>
	 * <i>native declaration : line 63</i>
	 */
	Pointer getClassDescription(PointerByReference det, int ClassID);
	/**
	 * Original signature : <code>int detect(detectNet*, char*, uint32_t, uint32_t, Result*, uint32_t)</code><br>
	 * <i>native declaration : line 65</i>
	 */
	int detect(PointerByReference det, ByteBuffer img, JetsonNanoLibrary.Result result, int overlay);
	/**
	 * Original signature : <code>int detect(detectNet*, char*, uint32_t, uint32_t, Result*, uint32_t)</code><br>
	 * <i>native declaration : line 65</i>
	 */
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
