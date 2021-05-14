
<?php
header('Content-Type: application/json');

$webcam = '/ram/webcam/';
$nexus = '/ram/433/Nexus-TH/231/';

if (file_exists($webcam) && file_exists($nexus)) {
    $json = array(
        'mtime' => str_replace(PHP_EOL, '', file_get_contents($webcam . '.mtime')),
        'temp' => str_replace(PHP_EOL, '', file_get_contents($nexus . 'temperature_C')) . '°C',
        'humi' => str_replace(PHP_EOL, '', file_get_contents($nexus . 'humidity')) . '%'
    );
    echo json_encode($json);
}
?>
