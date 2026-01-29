import math

def inverse_kinematics(x, y, l1, l2):
    # 1. Tính khoảng cách từ gốc đến điểm (x, y)
    d_squared = x**2 + y**2
    d = math.sqrt(d_squared)

    # Kiểm tra xem điểm có nằm ngoài tầm với không
    if d > (l1 + l2) or d < abs(l1 - l2):
        print("Lỗi: Điểm nằm ngoài vùng làm việc của robot!")
        return None

    # 2. Tính theta2 (Sử dụng định lý hàm số Cosine)
    # cos(theta2) = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)
    cos_theta2 = (d_squared - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Có 2 nghiệm: Elbow Up và Elbow Down
    theta2_elbow_down = math.acos(cos_theta2)
    theta2_elbow_up = -math.acos(cos_theta2)

    # 3. Tính theta1
    # theta1 = atan2(y, x) - atan2(l2*sin(theta2), l1 + l2*cos(theta2))
    alpha = math.atan2(y, x)
    
    def calculate_theta1(t2):
        beta = math.atan2(l2 * math.sin(t2), l1 + l2 * math.cos(t2))
        return alpha - beta

    t1_down = calculate_theta1(theta2_elbow_down)
    t1_up = calculate_theta1(theta2_elbow_up)

    return {
        "Elbow Down": (t1_down, theta2_elbow_down),
        "Elbow Up": (t1_up, theta2_elbow_up)
    }

# --- Chạy thử ---
L1 = 0.25  # Chiều dài khâu 1
L2 = 0.275  # Chiều dài khâu 2
target_x = 0.368
target_y = 0.351

result = inverse_kinematics(target_x, target_y, L1, L2)

if result:
    for mode, angles in result.items():
        t1, t2 = angles
        print(f"{mode}: Theta 1 = {t1:.2f}°, Theta 2 = {t2:.2f}°")