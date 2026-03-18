std_msgs/Header header

int32 vehicle_id     # 어떤 차량으로부터 오는 정보인지 (1, 2, 3, 4)
int32 zone_id        # 현재 진입하려는 위험 구역 ID(진입 전이면 -1)
int32 in_danger      # 현재 monitoring_zone 내부에 차량 만나면 1
float32 eta          # conflict_start_idx까지 도달 예상 시간 (초)
float32 velocity     # 현재 차량의 선속도 (m/s)
int32 lap            # 회전한 수
float32 x            # 해당 차량의 x 좌표
float32 y            
uint8 flag           # 가중치/우선순위용 플래그