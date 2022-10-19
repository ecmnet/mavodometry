package com.comino.mavodometry.estimators.inference;

public class YoloDetection {
	
	private static final String[] labelMap = {
			"Person",         "bicycle",    "Car",           "motorbike",     "Aeroplane",   "bus",           "train",
			"Truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
			"Bird",           "Cat",        "Dog",           "Horse",         "Sheep",       "Cow",           "elephant",
			"bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
			"suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
			"baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
			"fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
			"orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
			"chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
			"laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
			"toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
			"teddy bear",     "hair drier", "toothbrush" };

	
	public int    id;
	public float  confidence = 0;
	public int    xmin=0;
	public int    xmax=0;
	public int    ymin=0;
	public int    ymax=0;


	public YoloDetection(int id, float confidence, float xmin, float xmax, float ymin, float ymax) {
		super();
		this.id = id;
		this.confidence = confidence;
		
		this.xmin = (int)(xmin * 416) + 112;
		this.xmax = (int)(xmax * 416) + 112;
		this.ymin = (int)(ymin * 416) + 32;
		this.ymax = (int)(ymax * 416) + 32;
	}
	
	public String getLabel() {
		return labelMap[id];
	}


	public String toString() {
		return getLabel()+": "+confidence+"["+xmin+","+xmax+","+ymin+","+ymax+"]";
	}




}