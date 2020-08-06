#define WEBCAM_START			"su -c \"/home/www/webcam/webcam-start.sh\" hje"
#define WEBCAM_START_RESET		"su -c \"/home/www/webcam/webcam-start.sh reset\" hje"
#define WEBCAM_STOP				"su -c \"/home/www/webcam/webcam-stop.sh\" hje"
#define WEBCAM_STOP_TIMELAPSE	"su -c \"/home/www/webcam/webcam-stop.sh timelapse &\" hje"

// webcam off: ↑earlier, ↓later
#define WEBCAM_SUNDOWN			3600

// webcam on: ↑later ↓earlier
#define WEBCAM_SUNRISE			3500

int webcam_init(void);
void webcam_close(void);
