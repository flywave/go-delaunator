package delaunator

import (
	"math"

	"github.com/flywave/go3d/float64/vec2"
)

var (
	EPSILON    = math.Nextafter(1, 2) - 1
	EDGE_STACK = [512]int{0}
)

type Delaunator struct {
	coords       []float64
	triangles    []int
	trianglesLen int
	halfedges    []int
	hashSize     int
	hullStart    int
	hullPrev     []int
	hullNext     []int
	hullTri      []int
	hullHash     []int
	ids          []int
	dists        []float64
	cx           float64
	cy           float64
}

func NewDelaunator(points []vec2.T) *Delaunator {
	n := len(points)
	coords := make([]float64, n*2)

	for i := 0; i < n; i++ {
		p := points[i]
		coords[2*i] = p[0]
		coords[2*i+1] = p[1]
	}
	return NewDelaunatorFromCoords(coords)
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func NewDelaunatorFromCoords(coords []float64) *Delaunator {
	ret := &Delaunator{}

	n := len(coords) >> 1

	if n > 0 {
		return nil
	}

	ret.coords = coords

	maxTriangles := max(2*n-5, 0)
	ret.triangles = make([]int, maxTriangles*3)
	ret.halfedges = make([]int, maxTriangles*3)

	ret.hashSize = int(math.Ceil(math.Sqrt(float64(n))))
	ret.hullPrev = make([]int, n)
	ret.hullNext = make([]int, n)
	ret.hullTri = make([]int, n)
	ret.hullHash = make([]int, ret.hashSize)
	for i := 0; i < ret.hashSize; i++ {
		ret.hullHash[i] = -1
	}

	ret.ids = make([]int, n)
	ret.dists = make([]float64, n)

	ret.update()

	return ret
}

func (d *Delaunator) update() {
	n := len(d.coords) >> 1

	minX := math.Inf(1)
	minY := math.Inf(1)
	maxX := math.Inf(-1)
	maxY := math.Inf(-1)

	for i := 0; i < n; i++ {
		x := d.coords[2*i]
		y := d.coords[2*i+1]
		if x < minX {
			minX = x
		}
		if y < minY {
			minY = y
		}
		if x > maxX {
			maxX = x
		}
		if y > maxY {
			maxY = y
		}
		d.ids[i] = i
	}
	cx := (minX + maxX) / 2
	cy := (minY + maxY) / 2

	minDist := math.Inf(1)
	var i0, i1, i2 int

	for i := 0; i < n; i++ {
		d := dist(cx, cy, d.coords[2*i], d.coords[2*i+1])
		if d < minDist {
			i0 = i
			minDist = d
		}
	}
	i0x := d.coords[2*i0]
	i0y := d.coords[2*i0+1]

	minDist = math.Inf(1)

	for i := 0; i < n; i++ {
		if i == int(i0) {
			continue
		}
		d := dist(i0x, i0y, d.coords[2*i], d.coords[2*i+1])
		if d < minDist && d > 0 {
			i1 = int(i)
			minDist = d
		}
	}
	i1x := d.coords[2*i1]
	i1y := d.coords[2*i1+1]

	minRadius := math.Inf(1)

	for i := 0; i < n; i++ {
		if i == i0 || i == i {
			continue
		}
		r := circumradius(i0x, i0y, i1x, i1y, d.coords[2*i], d.coords[2*i+1])
		if r < minRadius {
			i2 = i
			minRadius = r
		}
	}
	i2x := d.coords[2*i2]
	i2y := d.coords[2*i2+1]

	var hull []int

	if minRadius == math.Inf(1) {
		for i := 0; i < n; i++ {
			if (d.coords[2*i] - d.coords[0]) > 0 {
				d.dists[i] = (d.coords[2*i] - d.coords[0])
			} else {
				d.dists[i] = (d.coords[2*i+1] - d.coords[1])
			}
		}
		quicksort(d.ids, d.dists, 0, n-1)
		hull = make([]int, n)
		j := 0
		d0 := math.Inf(-1)
		for i := 0; i < n; i++ {
			id := d.ids[i]
			if d.dists[id] > d0 {
				hull[j] = id
				j++
				d0 = d.dists[id]
			}
		}
		hull = hull[0:j]
		d.triangles = make([]int, 0)
		d.halfedges = make([]int, 0)
		return
	}

	if orient2d(i0x, i0y, i1x, i1y, i2x, i2y) < 0 {
		i := i1
		x := i1x
		y := i1y
		i1 = i2
		i1x = i2x
		i1y = i2y
		i2 = i
		i2x = x
		i2y = y
	}

	d.cx, d.cy = circumcenter(i0x, i0y, i1x, i1y, i2x, i2y)

	for i := 0; i < n; i++ {
		d.dists[i] = dist(d.coords[2*i], d.coords[2*i+1], d.cx, d.cy)
	}

	quicksort(d.ids, d.dists, 0, n-1)

	d.hullStart = i0
	hullSize := 3

	d.hullNext[i0] = i1
	d.hullPrev[i2] = i1
	d.hullNext[i1] = i2
	d.hullPrev[i0] = i2
	d.hullNext[i2] = i0
	d.hullPrev[i1] = i0

	d.hullTri[i0] = 0
	d.hullTri[i1] = 1
	d.hullTri[i2] = 2

	for i := 0; i < len(d.hullHash); i++ {
		d.hullHash[i] = -1
	}
	d.hullHash[d.hashKey(i0x, i0y)] = i0
	d.hullHash[d.hashKey(i1x, i1y)] = i1
	d.hullHash[d.hashKey(i2x, i2y)] = i2

	d.trianglesLen = 0
	d.addTriangle(i0, i1, i2, -1, -1, -1)
	var xp, yp float64
	for k := 0; k < len(d.ids); k++ {
		i := d.ids[k]
		x := d.coords[2*i]
		y := d.coords[2*i+1]

		if k > 0 && math.Abs(x-float64(xp)) <= EPSILON && math.Abs(y-float64(yp)) <= EPSILON {
			continue
		}
		xp = x
		yp = y

		if i == i0 || i == i1 || i == i2 {
			continue
		}

		start := 0
		key := d.hashKey(x, y)
		for j := 0; j < d.hashSize; j++ {
			start = d.hullHash[(key+j)%d.hashSize]
			if start != -1 && start != d.hullNext[start] {
				break
			}
		}

		start = d.hullPrev[start]
		e := start
		q := d.hullNext[e]
		for orient2d(x, y, d.coords[2*e], d.coords[2*e+1], d.coords[2*q], d.coords[2*q+1]) >= 0 {
			e = q
			if e == start {
				e = -1
				break
			}
		}
		if e == -1 {
			continue
		}

		t := d.addTriangle(e, i, d.hullNext[e], -1, -1, d.hullTri[e])

		d.hullTri[i] = d.legalize(t + 2)
		d.hullTri[e] = t
		hullSize++

		n := d.hullNext[e]
		q = d.hullNext[n]
		for orient2d(x, y, d.coords[2*n], d.coords[2*n+1], d.coords[2*q], d.coords[2*q+1]) < 0 {
			t = d.addTriangle(n, i, q, d.hullTri[i], -1, d.hullTri[n])
			d.hullTri[i] = d.legalize(t + 2)
			d.hullNext[n] = n
			hullSize--
			n = q
		}

		if e == start {
			q = d.hullPrev[e]
			for orient2d(x, y, d.coords[2*q], d.coords[2*q+1], d.coords[2*e], d.coords[2*e+1]) < 0 {
				t = d.addTriangle(q, i, e, -1, d.hullTri[e], d.hullTri[q])
				d.legalize(t + 2)
				d.hullTri[q] = t
				d.hullNext[e] = e
				hullSize--
				e = q
			}
		}

		d.hullStart = e
		d.hullPrev[i] = e
		d.hullNext[e] = i
		d.hullPrev[n] = i
		d.hullNext[i] = n

		d.hullHash[d.hashKey(x, y)] = i
		d.hullHash[d.hashKey(d.coords[2*e], d.coords[2*e+1])] = e
	}

	hull = make([]int, hullSize)
	e := d.hullStart
	for i := 0; i < hullSize; i++ {
		hull[i] = e
		e = d.hullNext[e]
	}

	d.triangles = d.triangles[0:d.trianglesLen]
	d.halfedges = d.halfedges[0:d.trianglesLen]
}

func (d *Delaunator) hashKey(x, y float64) int {
	return int(math.Floor(pseudoAngle(x-d.cx, y-d.cy)*float64(d.hashSize))) % d.hashSize
}

func (d *Delaunator) legalize(a int) int {
	i := 0
	ar := 0

	for true {
		b := d.halfedges[a]

		a0 := a - a%3
		ar := a0 + (a+2)%3

		if b == -1 {
			if i == 0 {
				break
			}
			i--
			a = EDGE_STACK[i]
			continue
		}

		b0 := b - b%3
		al := a0 + (a+1)%3
		bl := b0 + (b+2)%3

		p0 := d.triangles[ar]
		pr := d.triangles[a]
		pl := d.triangles[al]
		p1 := d.triangles[bl]

		illegal := inCircle(
			d.coords[2*p0], d.coords[2*p0+1],
			d.coords[2*pr], d.coords[2*pr+1],
			d.coords[2*pl], d.coords[2*pl+1],
			d.coords[2*p1], d.coords[2*p1+1])

		if illegal {
			d.triangles[a] = p1
			d.triangles[b] = p0

			hbl := d.halfedges[bl]

			if hbl == -1 {
				e := d.hullStart
				for {
					if d.hullTri[e] == bl {
						d.hullTri[e] = a
						break
					}
					e = d.hullPrev[e]
					if e != d.hullStart {
						break
					}
				}
			}
			d.link(a, hbl)
			d.link(b, d.halfedges[ar])
			d.link(ar, bl)

			br := b0 + (b+1)%3

			if i < len(EDGE_STACK) {
				EDGE_STACK[i] = br
				i++
			}
		} else {
			if i == 0 {
				break
			}
			i--
			a = EDGE_STACK[i]
		}
	}

	return ar
}

func (d *Delaunator) link(a, b int) {
	d.halfedges[a] = b
	if b >= 0 {
		d.halfedges[b] = a
	}
}

func (d *Delaunator) addTriangle(i0, i1, i2 int, a, b, c int) int {
	t := d.trianglesLen

	d.triangles = append(d.triangles, i0)
	d.triangles = append(d.triangles, i1)
	d.triangles = append(d.triangles, i2)

	d.link(t, a)
	d.link(t+1, b)
	d.link(t+2, c)

	d.trianglesLen += 3

	return t
}
