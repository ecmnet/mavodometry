//package com.comino.mavodometry.video.util;
//
//import org.bytedeco.javacpp.Pointer;
//import org.bytedeco.javacpp.PointerPointer;
//
//import org.bytedeco.cuda.cudart.*;
//import org.bytedeco.cuda.nvjpeg.*;
//import org.bytedeco.javacpp.Pointer;
//import org.bytedeco.javacpp.SizeTPointer;
//import org.bytedeco.javacpp.BytePointer;
//import org.bytedeco.javacpp.PointerPointer;
//
//import static org.bytedeco.cuda.global.cudart.*;
//import static org.bytedeco.cuda.global.nvjpeg.*;
//
//public class CudaJPEGEncoder {
//	
//	static class dev_malloc extends tDevMalloc {
//        final static dev_malloc instance = new dev_malloc().retainReference();
//
//        @Override
//        public int call(PointerPointer pointerPointer, long l) {
//            return cudaMalloc(pointerPointer, l);
//        }
//    }
//
//    static class dev_free extends tDevFree {
//        final static dev_free instance = new dev_free().retainReference();
//
//        @Override
//        public int call(Pointer pointer) {
//            return cudaFree(pointer);
//        }
//    }
//
//    static class host_malloc extends tPinnedMalloc {
//        final static host_malloc instance = new host_malloc().retainReference();
//
//        @Override
//        public int call(PointerPointer pointerPointer, long l, int i) {
//            return cudaHostAlloc(pointerPointer, l, i);
//        }
//    }
//
//    static class host_free extends tPinnedFree {
//        final static host_free instance = new host_free().retainReference();
//
//        @Override
//        public int call(Pointer pointer) {
//            return cudaFreeHost(pointer);
//        }
//    }
//
//    public static void CHECK_CUDA(String functionName, int result) {
//        if (result != CUDA_SUCCESS) {
//            throw new IllegalStateException(String.format("%s returned '%d'", functionName, result));
//        }
//    }
//
//    public static void CHECK_NVJPEG(String functionName, int result) {
//        if (result != NVJPEG_STATUS_SUCCESS) {
//            throw new IllegalStateException(String.format("%s returned '%d'", functionName, result));
//        }
//    }
//
//}
