package com.comino.mavodometry.utils;


public class TimeHysteresis {

	public final static int EDGE_BOTH    = 0; // hysteresis on both edges
	public final static int EDGE_RISING  = 1; // hysteresis on rising edges only
	public final static int EDGE_FALLING = 2; // hysteresis on falling edges only


	private int       gate_ms     = 0;
	private long      trigger     = 0;   
	private boolean   state       = false;
	private boolean   pre_state   = false;
	private int       edge        = 0;

	private Runnable  rising_action  = null;
	private Runnable  falling_action = null;


	public TimeHysteresis(float gate_sec) {
		this(gate_sec,EDGE_BOTH);
	}

	public TimeHysteresis(float gate_sec,int edge) {
		this.gate_ms = (int)(gate_sec * 1000.0f);
		this.state   = false;
		this.edge    = edge;
	}

	public void registerAction(int edge,Runnable r) {
		switch(edge) {
		case EDGE_BOTH:
			this.rising_action  = r;
			this.falling_action = r;
			break;
		case EDGE_RISING:
			this.rising_action = r;
			break;
		case EDGE_FALLING:
			this.falling_action = r;
			break;
		}
	}


	public boolean check(boolean condition) {
		if(pre_state != condition) {
			if((!condition && edge == EDGE_RISING) || (condition && edge == EDGE_FALLING)) {
				if(condition & !state && rising_action != null)
					rising_action.run();
				if(!condition & state && falling_action != null)
					falling_action.run();
				state = pre_state = condition;
				trigger = 0;
				return state;
			}
			pre_state = condition;
			trigger   = System.currentTimeMillis() + gate_ms;
			return state;
		}

		if(trigger > 0 && (System.currentTimeMillis() > trigger)) {
			if(pre_state & !state && rising_action != null)
				rising_action.run();
			if(!pre_state & state && falling_action != null)
				falling_action.run();
			state      = pre_state;
			trigger    = 0;
		}
		return state;
	}

	public boolean get() {
		return state;
	}

	public void reset() {
		trigger   = 0;
		pre_state = false;
		state     = false;
	}

	public static void main(String[] args)  {

		TimeHysteresis h = new TimeHysteresis(5.0f, TimeHysteresis.EDGE_BOTH);
		h.registerAction(EDGE_RISING, () -> System.out.println("Rising"));
		h.registerAction(EDGE_FALLING,() -> System.out.println("Falling"));

		long tms = System.currentTimeMillis();

		while(!h.get()) {
			try {
				h.check(true);
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		System.out.println();
		System.out.println(System.currentTimeMillis()-tms);

		while(h.get()) {
			try {
				h.check(false);
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		System.out.println();
		System.out.println(System.currentTimeMillis()-tms);

	}

}
