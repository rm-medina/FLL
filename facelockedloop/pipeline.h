struct detector;
struct pipeline {
	struct list stages;
	int status;
};

void pipeline_init(struct pipeline *pipe);
void pipeline_teardown(struct pipeline* pipe);
int pipeline_run(struct pipeline* pipe);


struct stage_ops {
	int (*initialize)(struct stage* step);
	int (*run)(struct stage* step);
	void (*teardown)(struct stage* step);
	int (*get_numbers)(struct stage* step);
};

struct stage {
	struct stage_ops *ops;
	struct stage_params *params;
	struct pipeline *pipeline;
	struct holder next;
	struct timespec duration;
	struct performance {
		struct timespec lastout;
		unsigned long nframes;
		unsigned long framesps;
	} stats;
};
