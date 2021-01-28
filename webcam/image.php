<?php

	function imageResize($url, $res) {
		// error_log($url, 0);
		if (strpos($url, "/") !== false || strpos($url, "..") !== false) {
			return;
		}

		// Content type
		header('Content-Type: image/jpeg');
		
		if ($res == 'l') {

			// Resize & Output
			$percent = 0.5;
			list($width, $height) = getimagesize($url);
			$new_width = $width * $percent;
			$new_height = $height * $percent;
			$image_p = imagecreatetruecolor($new_width, $new_height);
			$image = imagecreatefromjpeg($url);

			imagecopyresampled($image_p, $image, 0, 0, 0, 0, $new_width, $new_height, $width, $height);
			imagejpeg($image_p, null, 100);

		} else {

			// Output
			$image = imagecreatefromjpeg($url);
			imagejpeg($image, null, 100);

		}
	}

	$method = $_SERVER['REQUEST_METHOD'];
	if ($method == 'GET') {
		imageResize($_GET['url'], $_GET['res']);
	} elseif ($method == 'POST') {
		imageResize($_POST['url'], $_POST['res']);
	}

?>
