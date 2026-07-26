package main

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"runtime"
	"sync"
	"time"

	"physx-go/physx"
)

//go:embed viewer.html
var viewerHTML []byte

// ── Commands ─────────────────────────────────────────────────────────────────

type cmdKind int

const (
	cmdGetState cmdKind = iota
	cmdSwitchScene
	cmdFire
	cmdAddStack
)

type command struct {
	kind                  cmdKind
	scene                 string
	px, py, pz, dx, dy, dz float32
	resp                  chan response
}

type response struct {
	err string
}

var cmdCh = make(chan command, 32)

// ── SSE clients ──────────────────────────────────────────────────────────────

type sseClient struct {
	ch   chan []byte
	done <-chan struct{}
}

var (
	sseClients   = make(map[*sseClient]struct{})
	sseMu        sync.Mutex
)

func broadcastSSE(data []byte) {
	sseMu.Lock()
	defer sseMu.Unlock()
	for c := range sseClients {
		select {
		case c.ch <- data:
		default:
			// client too slow, skip
		}
	}
}

// ── JSON types ───────────────────────────────────────────────────────────────

type vec3JSON struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
	Z float32 `json:"z"`
}
type quatJSON struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
	Z float32 `json:"z"`
	W float32 `json:"w"`
}
type actorJSON struct {
	ID         int       `json:"id"`
	Type       string    `json:"type"`
	Hx         float32   `json:"hx"`
	Hy         float32   `json:"hy"`
	Hz         float32   `json:"hz"`
	Position   vec3JSON  `json:"position"`
	Rotation   quatJSON  `json:"rotation"`
	IsSleeping bool      `json:"isSleeping"`
}
type contactJSON struct {
	Position vec3JSON `json:"position"`
	Normal   vec3JSON `json:"normal"`
}
type stateJSON struct {
	Timestamp float64       `json:"timestamp"`
	Actors    []actorJSON   `json:"actors"`
	Contacts  []contactJSON `json:"contacts"`
}

// ── Simulation goroutine ─────────────────────────────────────────────────────

func runSimulation() {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()

	var s *snippetState
	var globalTime float64
	dt := float32(1.0 / 60.0)
	ticker := time.NewTicker(time.Duration(dt * float32(time.Second)))
	defer ticker.Stop()

	s = createHelloWorld()

	var simTick int
	for {
		select {
		case <-ticker.C:
			s.scene.Simulate(dt)
			globalTime += float64(dt)
			s.time = globalTime

			// Build state JSON every 3 ticks (~20 Hz) and push to SSE clients
			simTick++
			if simTick%3 == 0 {
				out := buildStateJSON(s)
				data, _ := json.Marshal(out)
				broadcastSSE(data)
			}

		case cmd := <-cmdCh:
			switch cmd.kind {
			case cmdGetState:
				cmd.resp <- response{}
			case cmdSwitchScene:
				clearHandles(s)
				s.release()
				switch cmd.scene {
				case "joints":
					s = createJointChains()
				case "contact":
					s = createContactReport()
				default:
					s = createHelloWorld()
				}
				simTick = 0
				cmd.resp <- response{}
			case cmdFire:
				fireProjectile(s, cmd.px, cmd.py, cmd.pz, cmd.dx, cmd.dy, cmd.dz)
				cmd.resp <- response{}
			case cmdAddStack:
				addNewStack(s)
				cmd.resp <- response{}
			}
		}
	}
}

func buildStateJSON(s *snippetState) stateJSON {
	out := stateJSON{
		Timestamp: s.time,
		Actors:    make([]actorJSON, 0, len(s.actors)),
		Contacts:  make([]contactJSON, 0, len(s.contacts)),
	}
	for i, ta := range s.actors {
		px, py, pz, qx, qy, qz, qw := ta.actor.GetGlobalPose()
		sleeping := ta.actor.IsSleeping()
		out.Actors = append(out.Actors, actorJSON{
			ID: i, Type: ta.geomType,
			Hx: ta.hx, Hy: ta.hy, Hz: ta.hz,
			Position:  vec3JSON{px, py, pz},
			Rotation:  quatJSON{qx, qy, qz, qw},
			IsSleeping: sleeping,
		})
	}
	for _, c := range s.contacts {
		out.Contacts = append(out.Contacts, contactJSON{
			Position: vec3JSON{c.pos.X, c.pos.Y, c.pos.Z},
			Normal:   vec3JSON{c.normal.X, c.normal.Y, c.normal.Z},
		})
	}
	return out
}

func fireProjectile(s *snippetState, px, py, pz, dx, dy, dz float32) {
	speed := float32(200.0)
	ball := physx.CreateDynamicSphere(s.physics, px, py, pz, 3.0, s.material, 10.0)
	ball.SetLinearVelocity(dx*speed, dy*speed, dz*speed)
	ball.SetAngularDamping(0.5)
	s.scene.AddActor(ball)
	s.actors = append(s.actors, trackedActor{ball, "sphere", 3.0, 0, 0})
}

func addNewStack(s *snippetState) {
	createStack(s, physx.NewTransform(0, 0, s.stackZ, 0, 0, 0, 1), 10, 2.0)
	s.stackZ -= 10.0
}

func clearHandles(s *snippetState) {
	for i := range s.actors {
		s.actors[i].actor.SetInvalid()
	}
	s.actors = nil
	for _, j := range s.joints {
		j.SetInvalid()
	}
	s.joints = nil
}

// ── HTTP handlers ────────────────────────────────────────────────────────────

func handleIndex(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	w.Header().Set("Cache-Control", "no-cache, no-store, must-revalidate")
	w.Write(viewerHTML)
}

// SSE stream — server pushes state to client
func handleStream(w http.ResponseWriter, r *http.Request) {
	flusher, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming not supported", 500)
		return
	}

	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")

	ch := make(chan []byte, 8)
	client := &sseClient{ch: ch, done: r.Context().Done()}

	sseMu.Lock()
	sseClients[client] = struct{}{}
	sseMu.Unlock()

	defer func() {
		sseMu.Lock()
		delete(sseClients, client)
		sseMu.Unlock()
	}()

	for {
		select {
		case data := <-ch:
			fmt.Fprintf(w, "data: %s\n\n", data)
			flusher.Flush()
		case <-r.Context().Done():
			return
		}
	}
}

func handleScene(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST required", 405)
		return
	}
	var req struct{ Scene string }
	json.NewDecoder(r.Body).Decode(&req)
	respCh := make(chan response, 1)
	cmdCh <- command{kind: cmdSwitchScene, scene: req.Scene, resp: respCh}
	<-respCh
	w.Write([]byte(`{"ok":true}`))
}

func handleFire(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST required", 405)
		return
	}
	var req struct{ Px, Py, Pz, Dx, Dy, Dz float32 }
	json.NewDecoder(r.Body).Decode(&req)
	respCh := make(chan response, 1)
	cmdCh <- command{kind: cmdFire,
		px: req.Px, py: req.Py, pz: req.Pz,
		dx: req.Dx, dy: req.Dy, dz: req.Dz,
		resp: respCh}
	<-respCh
	w.Write([]byte(`{"ok":true}`))
}

func handleAddStack(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST required", 405)
		return
	}
	respCh := make(chan response, 1)
	cmdCh <- command{kind: cmdAddStack, resp: respCh}
	<-respCh
	w.Write([]byte(`{"ok":true}`))
}

func main() {
	go runSimulation()

	http.HandleFunc("/", handleIndex)
	http.HandleFunc("/api/stream", handleStream)
	http.HandleFunc("/api/scene", handleScene)
	http.HandleFunc("/api/fire", handleFire)
	http.HandleFunc("/api/addstack", handleAddStack)

	port := ":8080"
	fmt.Printf("PhysX 3.4 Web Viewer — http://localhost%s\n", port)
	log.Fatal(http.ListenAndServe(port, nil))
}
