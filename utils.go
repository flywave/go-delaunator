package delaunator

import "math"

func pseudoAngle(dx, dy float64) float64 {
	p := dx / (math.Abs(dx) + math.Abs(dy))
	if dy > 0 {
		return 3 - p
	} else {
		return (1 + p) / 4
	}
}

func dist(ax, ay, bx, by float64) float64 {
	dx := ax - bx
	dy := ay - by
	return dx*dx + dy*dy
}

func inCircle(ax, ay, bx, by, cx, cy, px, py float64) bool {
	dx := ax - px
	dy := ay - py
	ex := bx - px
	ey := by - py
	fx := cx - px
	fy := cy - py

	ap := dx*dx + dy*dy
	bp := ex*ex + ey*ey
	cp := fx*fx + fy*fy

	return dx*(ey*cp-bp*fy)-
		dy*(ex*cp-bp*fx)+
		ap*(ex*fy-ey*fx) < 0
}

func circumradius(ax, ay, bx, by, cx, cy float64) float64 {
	dx := bx - ax
	dy := by - ay
	ex := cx - ax
	ey := cy - ay

	bl := dx*dx + dy*dy
	cl := ex*ex + ey*ey
	d := 0.5 / (dx*ey - dy*ex)

	x := (ey*bl - dy*cl) * d
	y := (dx*cl - ex*bl) * d

	return x*x + y*y
}

func circumcenter(ax, ay, bx, by, cx, cy float64) (x, y float64) {
	dx := bx - ax
	dy := by - ay
	ex := cx - ax
	ey := cy - ay

	bl := dx*dx + dy*dy
	cl := ex*ex + ey*ey
	d := 0.5 / (dx*ey - dy*ex)

	x = ax + (ey*bl-dy*cl)*d
	y = ay + (dx*cl-ex*bl)*d

	return
}

func quicksort(ids []int, dists []float64, left, right int) {
	if right-left <= 20 {
		for i := left + 1; i <= right; i++ {
			temp := ids[i]
			tempDist := dists[temp]
			j := i - 1
			for j >= left && dists[ids[j]] > tempDist {
				ids[j+1] = ids[j]
				j--
			}
			ids[j+1] = temp
		}
	} else {
		median := (left + right) >> 1
		i := left + 1
		j := right
		swap(ids, median, i)
		if dists[ids[left]] > dists[ids[right]] {
			swap(ids, left, right)
		}
		if dists[ids[i]] > dists[ids[right]] {
			swap(ids, i, right)
		}
		if dists[ids[left]] > dists[ids[i]] {
			swap(ids, left, i)
		}

		temp := ids[i]
		tempDist := dists[temp]
		for true {
			for true {
				i++
				if dists[ids[i]] < tempDist {
					break
				}
			}
			for true {
				j--
				if dists[ids[j]] > tempDist {
					break
				}
			}
			if j < i {
				break
			}
			swap(ids, i, j)
		}
		ids[left+1] = ids[j]
		ids[j] = temp

		if right-i+1 >= j-left {
			quicksort(ids, dists, i, right)
			quicksort(ids, dists, left, j-1)
		} else {
			quicksort(ids, dists, left, j-1)
			quicksort(ids, dists, i, right)
		}
	}
}

func swap(arr []int, i, j int) {
	tmp := arr[i]
	arr[i] = arr[j]
	arr[j] = tmp
}

var Orientation = map[string]int{
	"CW":       1,
	"CCW":      -1,
	"COLINEAR": 0,
}

func orient2d(ax, ay, bx, by, cx, cy float64) int {

	detleft := (ax - cx) * (by - cy)
	detright := (ay - cy) * (bx - cx)

	val := detleft - detright

	if math.Abs(val) < EPSILON {
		return Orientation["COLLINEAR"]
	} else if val > 0 {
		return Orientation["CCW"]
	}

	return Orientation["CW"]
}
