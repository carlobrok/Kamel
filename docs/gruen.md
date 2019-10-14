# gruen

Dokumentation für alle Methoden enthalten in der Datei **gruen.cpp** / **gruen.h**

## void set_gruen_data

```cpp
void set_gruen_data(Point & grcenter, int & grstate);
```

**Beschreibung:**

**parameters** | **possible values**
---------------|--------------------
grcenter       | cv::Point; 0 <= x < width; 0 <= y < height
grstate        | GRUEN_NICHT, GRUEN_BEIDE, GRUEN_LINKS, GRUEN_RECHTS
</br>

## void get_gruen_data
```cpp
void get_gruen_data(Point & grcenter, int & grstate);
```

**Beschreibung:**

</br>


## Point gruen_center

```cpp
Point gruen_center(vector <Point> & cont1, vector <Point> & cont2);

Point gruen_center(Point & p1, Point & p2);

Point gruen_center(vector <Point> & contour);
```

**Beschreibung:**


## Point rotate_point
```cpp
Point rotate_point(Point & origin_point, Point & rotating_point, float rotation, float length_factor);
```

**Beschreibung:**

**parameters** | **possible values**
-------------|--------------------
rotation      | should be >= -180 <= 180, can be any value
length_factor | should be > 0.0, can be any value, if < 0 the line is mirrored

</br>



## Point rotated_point_lenght
```cpp
Point rotated_point_lenght(Point & origin_point, float rotation, float length);
```

**Beschreibung:**

**parameters** | **possible values**
---------------|--------------------
rotation       | should be >= -180 <= 180, can be any value
length         | should be > 0, can be any value

</br>


## int gruen_check_normal
```cpp
int gruen_check_normal(Mat & img_rgb, Mat & bin_sw, Mat & bin_gr, vector<Point> & contour);
```

**Beschreibung:**

## void separate_gruen
```cpp
void separate_gruen(Mat & hsv, Mat & bin_gr);
```

**Beschreibung:**

## void gruen_calc
```cpp
void gruen_calc(Mat & img_rgb, Mat & img_hsv, Mat & bin_sw, Mat & bin_gr);
```

**Beschreibung:**


## float ratio_black_points
```cpp
float ratio_black_points(Point & origin, Point & destination, Mat & bin_sw, Mat & bin_gr, Mat & img_rgb);
```

**Beschreibung:**
