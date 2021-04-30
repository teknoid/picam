
<?php
header('Content-Type: application/json');

$webcam = '/ram/webcam/';
$nexus = '/ram/NEXUS/231/0/';

if (file_exists($webcam) && file_exists($nexus)) {
    $json = array(
        'mtime' => str_replace(PHP_EOL, '', file_get_contents($webcam . '.mtime')),
        'temp' => 'Temp ' . str_replace(PHP_EOL, '', file_get_contents($nexus . 'temp')) . '°C',
        'humi' => 'Hum ' . str_replace(PHP_EOL, '', file_get_contents($nexus . 'humi')) . '%'
    );
    echo json_encode($json);
}
?>
