package com.comino.mavodometry.concurrency;

import java.util.concurrent.Callable;
import java.util.concurrent.ForkJoinPool;



public class OdometryPool {

	private static ForkJoinPool pool = new ForkJoinPool(3);

	public static int getMaxThreads() {
		return pool.getParallelism();
	}

	public static <T> void submit(Callable<T> task) {
		if(pool.getRunningThreadCount() >= pool.getParallelism()) {
			System.out.println("No mor threads available");
		}
		pool.submit(task);
	}

	public static void submit(Thread task) {
		if(pool.getRunningThreadCount() >= pool.getParallelism()) {
			System.out.println("No more threads available");
		}
		pool.submit(task);

	}

	public static void close() {
		pool.shutdownNow();
	}

}
