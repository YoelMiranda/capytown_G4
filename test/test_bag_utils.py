"""Pruebas de las funciones puras de bag_utils (no requieren ROS ni un bag)."""
import math

import pytest

from capytown_esan.bag_utils import metricas_giro, metricas_recta, unwrap


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _giro(total_deg, n=200):
    """theta envuelto en (-pi, pi] de un giro en sitio de total_deg."""
    return [_wrap(math.radians(total_deg) * i / n) for i in range(n + 1)]


def test_recta_1m_con_desvio_3cm():
    n = 100
    x = [1.0 * i / n for i in range(n + 1)]
    y = [0.03 * i / n for i in range(n + 1)]
    th = [0.0] * (n + 1)
    m = metricas_recta(x, y, th)
    assert m['d_odom'] == pytest.approx(1.0004, abs=1e-4)
    assert m['desvio_lateral'] == pytest.approx(0.03, abs=1e-9)
    assert m['avance'] == pytest.approx(1.0, abs=1e-9)
    assert m['dtheta_deg'] == pytest.approx(0.0, abs=1e-9)


def test_recta_hacia_el_oeste_desvio_a_la_derecha():
    # heading inicial pi (este -> oeste, como A2); el robot termina 3 cm al norte,
    # que para quien mira al oeste es la derecha -> desvio negativo
    x = [0.0, -0.5, -1.0]
    y = [0.0, 0.015, 0.03]
    th = [math.pi, math.pi, -math.pi]
    m = metricas_recta(x, y, th)
    assert m['avance'] == pytest.approx(1.0, abs=1e-9)
    assert m['desvio_lateral'] == pytest.approx(-0.03, abs=1e-9)
    assert m['dtheta_deg'] == pytest.approx(0.0, abs=1e-9)


def test_unwrap_quita_saltos():
    th = _giro(360.0)
    u = unwrap(th)
    assert all(abs(b - a) < math.pi for a, b in zip(u, u[1:]))
    assert u[-1] - u[0] == pytest.approx(2.0 * math.pi, abs=1e-9)


@pytest.mark.parametrize('total', [360.0, -360.0, 720.0, -720.0])
def test_giro_acumulado_con_signo(total):
    th = _giro(total)
    # el giro pasa por +-pi, donde theta envuelto salta
    assert any(abs(b - a) > math.pi for a, b in zip(th, th[1:]))
    assert metricas_giro(th) == pytest.approx(total, abs=1e-6)


def test_metricas_con_un_punto_fallan():
    with pytest.raises(ValueError):
        metricas_recta([0.0], [0.0], [0.0])
    with pytest.raises(ValueError):
        metricas_giro([0.0])
