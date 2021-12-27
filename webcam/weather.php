<?php
header('Content-Type: application/json');

$mtime = preg_replace("/\r|\n/", "", file_get_contents('/ram/webcam/.mtime'));
$temp  = preg_replace("/\r|\n/", "", file_get_contents('/ram/433/Nexus-TH/53/temperature_C'));
$humi  = preg_replace("/\r|\n/", "", file_get_contents('/ram/433/Nexus-TH/53/humidity'));
$lumi  = preg_replace("/\r|\n/", "", file_get_contents('/ram/MCP3204/lum_percent'));
$baro  = preg_replace("/\r|\n/", "", file_get_contents('/ram/433/RadioHead-ASK/0/payload/2'));

$baro = ($baro + 101325) / 100;

$json = array(
	'mtime' => $mtime,
	'temp'  => $temp . ' °C',
	'humi'  => $humi . ' %',
	'lumi'  => $lumi . ' %',
	'baro'  => $baro . ' hPa'
);
echo json_encode($json);
?>
