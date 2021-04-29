
<?php
header('Content-Type: application/json');

$json = array(
    'temp' => 'Temp ' . str_replace(PHP_EOL, '', file_get_contents('/ram/NEXUS/231/0/temp')) . '°C',
    'humi' => 'Hum ' . str_replace(PHP_EOL, '', file_get_contents('/ram/NEXUS/231/0/humi')) . '%'
);

echo json_encode($json);
?>
