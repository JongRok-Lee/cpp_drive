
# 자율주행 데브코스 1차 오프라인 프로젝트를 위한 C++ 주행 예제 코드  
특이사항: 제어파트가 아닌 인지 파트에 집중하고자 기본 제공된 Python 예제 코드를 수정
- Python 코드 예시 (slope 와 pos를 활용한 필터링)
    ```
        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])
    ```
- 변경된 C++ 코드 예시 (예외 사항 처리를 위한 변경된 필터링)
    ```
        if (std::abs(slope) <= slope_range) {
            slopes.push_back(slope);
            new_lines.push_back(line);
        }
    ```
